#pragma once
/*
 Created:	11.11.2025
 Authors:	Jonathan Ploner
*/
#include <Arduino.h>
#include <configuration.h>
#include <vl53l4cd_api.h> 
#include <vl53l4cd_class.h> 
#include <VL6180X.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

uint8_t status;
VL53L4CD_Result_t results;
uint8_t NewDataReady;
sh2_SensorValue_t sensorValue;
sh2_SensorValue_t gyro_value;
sh2_SensorValue_t acceler_value;
sh2_SensorValue_t game_value;
bool gyro = false;
bool accel = false;
bool game = false;
int vl53l4cd_xshut = 30;
int vl6180x_xshut = 34;
int vl53l4cd2_xshut = 38;