#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "controller.h"

LiquidCrystal_I2C lcd(0x27,16,2);
WiFiClient espClient;
PubSubClient client(espClient);

// setup ssid, password, and serverIP
const char* ssid = "";
const char* password = "";
const char* mqtt_server = ""; // 192.168.69.10
const char* clientName = "controller";

int Kp, Ki, Kd;
char arryForKp[10], arryForKi[10], arryForKd[10];
unsigned long messageTimestamp = 0;

void reconnect()  {
    while (!client.connected())  {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(clientName)) {
            Serial.println("connected");
            client.subscribe("test"); 
        }
        else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    beginLCD(lcd);
    delay(3000);
    setupWiFi(ssid, password);
    client.setServer(mqtt_server, 1883);
    lcd.clear();
    lcd.backlight();
}

void loop() {
    uint64_t now = millis();
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    getValue(&Kp, &Ki, &Kd, 255, 255, 255);
    printLCD(lcd, Kp, Ki, Kd);
    
   if(now - messageTimestamp > 500) {
        messageTimestamp = now;
        sprintf(arryForKp, "Kp : %d", Kp);
        sprintf(arryForKi, "Ki : %d", Ki);
        sprintf(arryForKd, "Kd : %d", Kd);

        client.publish("test", arryForKp);
        client.publish("test", arryForKi);
        client.publish("test", arryForKd);

        lcd.clear(); 
    }
}