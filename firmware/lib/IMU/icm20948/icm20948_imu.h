#ifndef IMU_H
#define IMU_H

#include "imu.h"
#include "Wire.h"
#include "ICM_20948.h"

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0

struct icm20948_imu_config {
    TwoWire &_i2c;
    uint8_t adr_val;

    icm20948_imu_config(TwoWire &_i2c) : _i2c(_i2c), adr_val(AD0_VAL) {};
};

class ICM20948_IMU : public IMU<icm20948_imu_config> {
    public:
        ICM_20948_I2C icm20948_imu;

        // Methods
        virtual bool initIMU(imu_config config);
        
        // Read data
        virtual void getData();
        virtual void sleep();
        virtual void clearInterrupt();

        // Store data
        float accl[3];
        float gyro[3];
        float magn[3];
};
#endif