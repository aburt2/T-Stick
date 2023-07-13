#ifndef IMU_H
#define IMU_H

//#include <stdint.h>
#include <Wire.h>
#include <SPI.h>
#include <sensor.h>
#include <SparkFunLSM9DS1.h>

class IMU : public sensor{
    public:
        bool initialise(uint8_t I2C_ADDR = 0x30);
        bool getSensorData();
        int getData(int data_index);
        LSM9DS1 imuSensor;      // for IMU board
        // Accelerometer data
        int16_t ax;
        int16_t ay;
        int16_t az;
        // Gyro data
        int16_t gx;
        int16_t gy;
        int16_t gz;
        // Magnet data
        int16_t mx;
        int16_t my;
        int16_t mz;
};

#endif
