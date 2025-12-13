#pragma once
#include <Arduino.h>
#include <vl53l4cd_api.h> 
#include <vl53l4cd_class.h> 
#include <VL6180X.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include "configuration.h"


class Midrange {
    public: 
    VL53L4CD front = VL53L4CD(&Wire, short_LF_xshutPin);
    VL53L4CD back = VL53L4CD(&Wire, mid_front_xshutPin);

    void init(VL53L4CD* sensor, uint8_t xshut, uint8_t i2c_address);
    int getrangemm(uint8_t type);
    
};

class Shortrange {
    public:
    VL6180X lf;
    VL6180X lb;
    VL6180X rf;
    VL6180X rb;

    void init(VL6180X* sensor, uint8_t xshut, uint8_t i2c_address);
    int getrengemm(uint8_t type);

};

class Gyro {
    public:   
        Adafruit_BNO08x bno08x;

        void init(Adafruit_BNO08x* sensor, uint8_t i2c_address);
        sh2_SensorValue_t getAngle(uint8_t type);

};

class Spectrum_sensor {
    public:
        Adafruit_AS7341 front;
        Adafruit_AS7341 back;
        void init(Adafruit_AS7341* sensor);
        int readSpektrum(uint8_t type, uint8_t channel);
        int getbw(uint8_t type);
        int getcolor(uint8_t type, char color);
};