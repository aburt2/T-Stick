#ifndef _ICM4670_MMC5633_IMU_H_
#define _ICM4670_MMC5633_IMU_H_

// Include IMU header
#include "imu.h"

// Include comms classes from Arduino
#include "Wire.h" // needed for magnetometer
#include "SPI.h"  // will probably be needed

// Include IMU classes from external libraries
#define MAG_ID 12345
#define MIN_DELTAT 10000 // 10000 microseconds (10ms)
#include "utils/IMU_utils.h"
#include "utils/MMC5633NJL_utils.h"

struct icm42670_mmc5633_config {
    TwoWire &_i2c;
    SPIClass &_spi;
    uint8_t ag_cs_pin;
    uint8_t mag_addr;

    // TODO: Set default CS pin and magnetometer address
    icm42670_mmc5633_config(SPIClass &_spi_class,TwoWire &_i2c_class) : _spi(_spi_class), _i2c(_i2c_class), ag_cs_pin(0), mag_addr(0) {};
};

class ICM42670_MMC5633_IMU : public IMU<icm42670_mmc5633_config> {
    public:
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

        // compass heading
        float heading;
        float yaw;
        float pitch;
        float roll;
        float deltaT;
        unsigned long lastTime;

        // Multipliers
        const float accelSensitivity =  1.0f / 2048.00f;
        const float gyroSensitivity = 16.40f;
        const float gyroMultipier = float(PI) / (gyroSensitivity * 180.0f);

    ICM42670_MMC5633_IMU() {
        lastTime = micros();
    }
    void updateOrientation();
    void magnetometerCalibration();
};
#endif