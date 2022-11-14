#include "controller.h"
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

void setupWiFi(const char* ssid, const char* password) {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    
    while(WiFi.status()!= WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi Connected");
}

void beginLCD(LiquidCrystal_I2C lcd) {
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("Connecting to ");
    lcd.setCursor(0,1);
    lcd.print("WiFi...");
}

void getValue(int *Kp, int *Ki, int *Kd, int valForKp, int valForKi, int valForKd) {
    *Kp=map(analogRead(KpPin), 0, 4095, 0, valForKp);
    *Ki=map(analogRead(KiPin), 0, 4095, 0, valForKi);
    *Kd=map(analogRead(KdPin), 0, 4095, 0, valForKd);
}

void printLCD(LiquidCrystal_I2C lcd, int Kp, int Ki, int Kd) {
    lcd.setCursor(0,0);
    lcd.print("Kp="); lcd.print(Kp);
    lcd.setCursor(9,0);
    lcd.print("Ki="); lcd.print(Ki);
    lcd.setCursor(0,1);
    lcd.print("Kd="); lcd.print(Kd);
}