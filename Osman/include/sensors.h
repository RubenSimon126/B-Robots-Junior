#include <Arduino.h>
#include <configuration.h>
#include <vl53l4cd_api.h> 
#include <vl53l4cd_class.h> 
#include <VL6180X.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>


class midrange {
    public: 
    VL53L4CD front;
    VL53L4CD back;

    void init(uint8_t xshut, uint8_t i2c_address);
    void stop(void);
    void setadress(uint8_t address);
    void getrangemm(void);
};