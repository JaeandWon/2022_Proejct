#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define KpPin 34
#define KiPin 32
#define KdPin 33

void setupWiFi(const char* ssid, const char* password);

void beginLCD(LiquidCrystal_I2C lcd);

void getValue(int *Kp, int *Ki, int *Kd, int valForKp, int valForKi, int valForKd);

void printLCD(LiquidCrystal_I2C lcd, int Kp, int Ki, int Kd);
#endif