#ifndef _LSM9DS1_IMU_H_
#define _LSM9DS1_IMU_H_

#include <imu.h>
#include <SparkFunLSM9DS1.h>

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0

struct lsm9ds1_config {
    TwoWire &_i2c;
    uint8_t ag_addr;
    uint8_t m_addr;

    lsm9ds1_config(TwoWire &_i2c) : _i2c(_i2c), ag_addr(LSM9DS1_AG_ADDR(1)), m_addr(LSM9DS1_M_ADDR(1)) {};
};

class LSM9DS1_IMU : public IMU<lsm9ds1_config> {
    public:
        LSM9DS1 lsm9ds1_imu;

        // Methods
        bool initIMU(imu_config config);
        
        // Read data
        void getData();
        void sleep();
        void clearInterrupt();

        // Store data
        float accl[3];
        float gyro[3];
        float magn[3];
};
#endif