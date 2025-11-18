#include <Arduino.h>
#include <configuration.h>
#include <vl53l4cd_api.h> 
#include <vl53l4cd_class.h> 
#include <VL6180X.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

VL53L4CD vl5314cd_sat(&Wire, vl53l4cd_xshut);
VL53L4CD vl5314cd2_sat(&Wire, vl53l4cd2_xshut);
VL6180X vl6180;
Adafruit_AS7341 as7341;
Adafruit_BNO08x bno08x;

int main(void){
    init();
    pinMode(vl6180x_xshut, OUTPUT);
    Wire.begin();
    Serial.begin(115200);
    while (!Serial) delay(10);

    if (!bno08x.begin_I2C(0x4A)) {
    Serial.println("Failed to find BNO08X chip");
    while (1) delay(10);
    }
    bno08x.enableReport(SH2_ACCELEROMETER);
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);

    if (!as7341.begin()) {
    Serial.println("Failed to find AS7341 sensor! Check wiring.");
    while (1) delay(10);
    }
    Serial.println("as7341 done");

    digitalWrite(vl6180x_xshut, LOW);
    vl5314cd2_sat.VL53L4CD_Off();
    vl5314cd_sat.begin();
    vl5314cd_sat.InitSensor(0x29);
    vl5314cd_sat.VL53L4CD_SetI2CAddress(0xA1);
    vl5314cd_sat.VL53L4CD_StartRanging();
    vl5314cd2_sat.VL53L4CD_On();
    vl5314cd2_sat.begin();
    vl5314cd2_sat.InitSensor(0x29);
    vl5314cd2_sat.VL53L4CD_SetI2CAddress(0xA3);
    vl5314cd2_sat.VL53L4CD_StartRanging();
    digitalWrite(vl6180x_xshut, HIGH);
    Serial.println("53 finished");
    delay(100);
    vl6180.init();
    vl6180.configureDefault();
    vl6180.setAddress(0xA2);
    vl6180.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    vl6180.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
    vl6180.setTimeout(500);
    vl6180.stopContinuous();
    delay(300);
    vl6180.startInterleavedContinuous(100);
    Serial.println("anderer finished");
 
    while (1){
        #pragma region vl5314cd
        NewDataReady = 0;
        do {
            status = vl5314cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
            Serial.print("-");
        } while (!NewDataReady);
        
        if ((!status) && (NewDataReady != 0)) {
            vl5314cd_sat.VL53L4CD_ClearInterrupt();
            vl5314cd_sat.VL53L4CD_GetResult(&results);
            Serial.print("VL53L4CD:\nStatus = ");
            Serial.print(results.range_status);
            Serial.print(", Distance = ");
            Serial.print(results.distance_mm);
            Serial.print(" mm, Signal = ");
            Serial.println(results.signal_per_spad_kcps);
        }
        #pragma endregion

        #pragma region vl5314cd2
        NewDataReady = 0;
        do {
            status = vl5314cd2_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
            Serial.print("-");
        } while (!NewDataReady);
        
        if ((!status) && (NewDataReady != 0)) {
            vl5314cd2_sat.VL53L4CD_ClearInterrupt();
            vl5314cd2_sat.VL53L4CD_GetResult(&results);
            Serial.print("VL53L4CD2:\nStatus = ");
            Serial.print(results.range_status);
            Serial.print(", Distance = ");
            Serial.print(results.distance_mm);
            Serial.print(" mm, Signal = ");
            Serial.println(results.signal_per_spad_kcps);
        }
        #pragma endregion

        #pragma region vl6180
        Serial.print("\nVL6180:\nAmbient: ");
        Serial.print(vl6180.readAmbientContinuous());
        if (vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

        Serial.print(" Range: ");
        Serial.print(vl6180.readRangeContinuousMillimeters());
        if (vl6180.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
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
        
        #pragma region BNO085
        Serial.println();
        while(!gyro || !accel || !game){
        if (bno08x.getSensorEvent(&sensorValue)) {
            switch (sensorValue.sensorId) {
                case SH2_ACCELEROMETER:
                    acceler_value = sensorValue;
                    accel = true;   
                    break;

                case SH2_GYROSCOPE_CALIBRATED:
                    gyro_value = sensorValue;
                    gyro = true;   
                    break;

                case SH2_GAME_ROTATION_VECTOR:
                    game_value = sensorValue;
                    game = true;   
                    break;
            }
        }}
        game=false;gyro=false;accel=false;
        Serial.print("Accel X: "); Serial.print(acceler_value.un.accelerometer.x);
        Serial.print(" Y: "); Serial.print(acceler_value.un.accelerometer.y);
        Serial.print(" Z: "); Serial.println(acceler_value.un.accelerometer.z);
        Serial.print("Gyro X: "); Serial.print(gyro_value.un.gyroscope.x);
        Serial.print(" Y: "); Serial.print(gyro_value.un.gyroscope.y);
        Serial.print(" Z: "); Serial.println(gyro_value.un.gyroscope.z);
        Serial.print("Quat i: "); Serial.print(game_value.un.gameRotationVector.i);
        Serial.print(" j: "); Serial.print(game_value.un.gameRotationVector.j);
        Serial.print(" k: "); Serial.print(game_value.un.gameRotationVector.k);
        #pragma endregion
        
        Serial.println("_________________________________________________________________________________\n"); 
        delay(1000);
    }
}