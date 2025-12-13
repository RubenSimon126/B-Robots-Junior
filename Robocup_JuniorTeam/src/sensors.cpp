#include "sensors.h"

void Midrange::init(VL53L4CD* sensor, uint8_t xshut, uint8_t i2c_address){
    sensor->begin();
    sensor->VL53L4CD_Off();
    sensor->InitSensor(0x29);
    sensor->VL53L4CD_SetI2CAddress(i2c_address);
	sensor->VL53L4CD_StartRanging();
}

int Midrange::getrangemm(uint8_t type){
    VL53L4CD* sensor = &front;
    if(type ==1) sensor = &back;

    uint8_t NewDataReady = 0;
    uint8_t status;
    VL53L4CD_Result_t results;
    do {
        status = sensor->VL53L4CD_CheckForDataReady(&NewDataReady);
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0)) {
            sensor->VL53L4CD_ClearInterrupt();
            sensor->VL53L4CD_GetResult(&results);
    }
    
    if (!results.range_status) return results.distance_mm;
    else return 1024;
}

void Shortrange::init(VL6180X* sensor, uint8_t xshut, uint8_t i2c_address){
    pinMode(xshut, INPUT_PULLUP);
    sensor->setTimeout(500);
    sensor->init();
    sensor->configureDefault();
    sensor->writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    sensor->writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
    sensor->setTimeout(500);
    delay(100);
    sensor->stopContinuous();
    sensor->setTimeout(500);
    delay(100);
    sensor->setAddress(i2c_address);
    sensor->startInterleavedContinuous(100);
    
}

int Shortrange::getrengemm(uint8_t type){
    VL6180X* sensor = &lf;
    if(type==1) VL6180X* sensor = &lb;
    if(type==2) VL6180X* sensor = &rf;
    if(type==3) VL6180X* sensor = &rb;

    return sensor->readRangeContinuousMillimeters();
}

void Gyro::init(Adafruit_BNO08x* sensor, uint8_t i2c_address){
    sensor->begin_I2C(i2c_address);
    //bno08x.enableReport(SH2_ACCELEROMETER);
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
    //sensor->enableReport(SH2_GYROSCOPE_CALIBRATED);
}

sh2_SensorValue_t Gyro::getAngle(uint8_t type){
    Adafruit_BNO08x* sensor = &bno08x;
    sh2_SensorValue_t game_value;

    while(!sensor->getSensorEvent(&game_value))delay(10);
    return game_value;
}

void Spectrum_sensor::init(Adafruit_AS7341* sensor){
    sensor->begin();
}

int Spectrum_sensor::getbw(uint8_t type){
    
}