#include <Arduino.h>
#include <vl53l4cd_api.h> 
#include <vl53l4cd_class.h> 
#include <VL6180X.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include <SparkFun_TB6612.h>
#include "configuration.h"
#include "sensors.h"

sh2_SensorValue_t sensorValue;
sh2_SensorValue_t gyro_value;
sh2_SensorValue_t acceler_value;
sh2_SensorValue_t game_value;
bool gyro = false;
bool accel = false;
bool game = false;
Midrange midrange;
Shortrange shortrange;
Adafruit_AS7341 as7341;
Gyro bno085;
Motor motor_lf = Motor(MotorT_LF_IN1, MotorT_LF_IN2, MotorT_LF_PWM, 1, MotorT_L_STBY);

int main(void){
    init();
    pinMode(short_LF_xshutPin, OUTPUT);
    Serial.begin(115200);
    Wire.begin();

    bno085.init(&bno085.bno08x, 0x4A);

    if (!as7341.begin()) {
        Serial.println("Failed to find AS7341 sensor!");
        while (1) delay(10);
    }
    Serial.println("as7341 done");

    digitalWrite(short_LF_xshutPin, LOW);
    delay(100);
    midrange.init(&midrange.front, mid_front_xshutPin, 0xA1);
    midrange.init(&midrange.back, mid_back_xshutPin, 0xA3);
    digitalWrite(short_LF_xshutPin, HIGH);
    Serial.println("Mid ToF finished");
    delay(100);
    shortrange.init(&shortrange.lf, short_LF_xshutPin, 0xA2);
    Serial.println("Short ToF finished");
 
    while (1){
        #pragma region Sensors
        Serial.print("Front Distance = ");
        Serial.println(midrange.getrangemm(0));

        Serial.print("Back Distance = ");
        Serial.println(midrange.getrangemm(1));

        Serial.print("Short Distance LF: ");
        Serial.println(shortrange.getrengemm(0));

        Serial.print("\nGyro\n i: "); Serial.print(bno085.getAngle(0).un.gameRotationVector.i);
        Serial.print(" j: "); Serial.print(bno085.getAngle(0).un.gameRotationVector.j);
        Serial.print(" k: "); Serial.print(bno085.getAngle(0).un.gameRotationVector.k);

        #pragma endregion

        #pragma region as7341
        if (!as7341.readAllChannels()){
            Serial.println("Not could read all channels!");
        }
        Serial.println("\n\nAS7341: ");
        Serial.print("Channel F1: "); Serial.println(as7341.readChannel(AS7341_ADC_CHANNEL_0));
        Serial.print("Channel F2: "); Serial.println(as7341.readChannel(AS7341_ADC_CHANNEL_1));
        Serial.print("Channel F3: "); Serial.println(as7341.readChannel(AS7341_ADC_CHANNEL_2));
        Serial.print("Channel F4: "); Serial.println(as7341.readChannel(AS7341_ADC_CHANNEL_3));
        Serial.print("Channel F5: "); Serial.println(as7341.readChannel(AS7341_ADC_CHANNEL_4));
        Serial.print("Channel F6: "); Serial.println(as7341.readChannel(AS7341_ADC_CHANNEL_5));
        #pragma endregion
        
        Serial.println("\n_________________________________________________________________________________\n"); 

        if (shortrange.getrengemm(0) <50) motor_lf.drive(100);
        else if (shortrange.getrengemm(0) <100) motor_lf.drive(-100);
        else motor_lf.brake();

        delay(1000);
    }
}