#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <PID_v1.h>
#include <Wire.h>
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL //아두이노에서 사용시 주석해제
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define IPp 4
#include <WiFi.h>
#include <SocketIOclient.h>
#include <PubSubClient.h>
#include <String.h>
#define PORT 3000
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double setpoint = 182;  
SocketIOclient socketIO;
WiFiClient espClient;
PubSubClient client(espClient);

const char* ssid = "jwipp"; // RATS_2.4G";
const char* password = "01027628569"; //"rats8005";
const char* serverAddr = "192.168.158.8";//"192.168.1.47";
const char* clientName = "Robot";

double Kp = 400, Ki = 0.8, Kd = 10, Gyro = 0;
String StringKp = "0", StringKi = "0", StringKd = "0";
char message[100];

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Serial.begin(115200);
    client.setServer(serverAddr, 1883);
    client.setCallback(callback);

    // server address, port,  URL
    socketIO.begin(serverAddr, PORT, "/socket.io/?EIO=4");

    // event handler
    socketIO.onEvent(socketIOEvent);

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
        Wire.setClock(400000L);
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(IPp, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();

    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(LED_PIN, OUTPUT);
    pinMode(19, OUTPUT);
    pinMode(18, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(17, OUTPUT);
    pinMode(4, OUTPUT);

    analogWrite(19, LOW);
    analogWrite(18, LOW);
    analogWrite(5, LOW);
    analogWrite(17, LOW);

    setupWiFi(ssid, password);
}
unsigned long messageTimestamp = 0;

void loop() {

    if (!dmpReady) return;                                                      

    input = ypr[1] * 180 / M_PI + 180;
    Gyro = input;
    fifoCount -= packetSize;
    pid.Compute();
    if (output > 0){       
        Forward();          
    }
    if (output < 0) {
        Reverse();
    }
      uint64_t now = millis();
    socketIO.loop();
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
    sprintf(message, "[\"startmsg\", {\"Kp\" : %lf, \"Ki\" : %lf, \"Kd\" : %lf, \"Gyro\": %lf}]",Kp, Ki, Kd,Gyro); 
    if(now - messageTimestamp > 1500) {
        messageTimestamp = now;
        // Send event 
        socketIO.sendEVENT(message);
    }


    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
         mpu.resetFIFO();
    } 
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);      
        mpu.dmpGetGravity(&gravity, &q);           
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180 / M_PI + 180;
        fifoCount -= packetSize;
    }

}
void setupWiFi(const char* ssid, const char* password){
    Serial.println("Connecting to ");
    WiFi.begin(ssid, password);
    //WiFi.disconnect();
    while(WiFi.status()!= WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi Connected");
}

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            Serial.printf("[IOc] Disconnected!\n");
            break;
        case sIOtype_CONNECT:
            Serial.printf("[IOc] Connected to url: %s\n", payload);

            // join default namespace (no auto join in Socket.IO V3)
            socketIO.send(sIOtype_CONNECT, "/");
            break;
        case sIOtype_ACK:
            Serial.printf("[IOc] get ack: %u\n", length);
            break;
        case sIOtype_ERROR:
            Serial.printf("[IOc] get error: %u\n", length);
            break;
        case sIOtype_BINARY_EVENT:
            Serial.printf("[IOc] get binary: %u\n", length);
            break;
        case sIOtype_BINARY_ACK:
            Serial.printf("[IOc] get binary ack: %u\n", length);
            break;
    }
}

void reconnect() 
{
    while (!client.connected())
    {
        if (client.connect(clientName))
        {

            client.subscribe("test"); 
        }
        else
        {
            delay(5000);
        }
    }
}

void callback(char *topic, byte* payload, unsigned int length)
{
    if ((char)payload[1] == 'p')
    {
        StringKp.clear();
        for (int i = 5; i < length; i++)
        {
            StringKp += (char)payload[i];
        }
        Kp = StringKp.toInt();
    }
    else if ((char)payload[1] == 'i')
    {
        StringKi.clear();
        for (int i = 5; i < length; i++)
        {
            StringKi += (char)payload[i];
        }
        Ki = StringKi.toInt();
    }
    else if ((char)payload[1] == 'd')
    {
        StringKd.clear();
        for (int i = 5; i < length; i++)
        {
            StringKd += (char)payload[i];
        }
        Kd = StringKd.toInt();
    }
}

  
void Forward() 
{
  analogWrite(17, output);
  analogWrite(5, 0);
  analogWrite(18, output);
  analogWrite(19, 0);
}

void Reverse() 
{
  analogWrite(17, 0);
  analogWrite(5, output * -1);
  analogWrite(18, 0);
  analogWrite(19, output * -1);
}
 