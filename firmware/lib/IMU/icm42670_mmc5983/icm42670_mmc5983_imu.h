#ifndef _ICM4670_MMC5983_IMU_H_
#define _ICM4670_MMC5983_IMU_H_

// Include IMU header
#include "imu.h"

// Include comms classes from Arduino
#include "SPI.h"  // will probably be needed

// Include IMU classes from external libraries
#include "ICM42670P.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h>

struct mems_config {
    gpio_num_t cipo_pin;
    gpio_num_t copi_pin;
    gpio_num_t sck_pin;
    gpio_num_t cs_pin;
    gpio_num_t int_pin;

    mems_config(gpio_num_t cipo, gpio_num_t copi, gpio_num_t sck, gpio_num_t cs, gpio_num_t _int_pin) : 
        cipo_pin(cipo), 
        copi_pin(copi),
        sck_pin(sck), 
        cs_pin(cs), 
        int_pin(_int_pin) {};
    
    // mems_config(mems_config &_config) : cipo_pin(0), copi_pin(0), sck_pin(0), cs_pin(0), int_pin(0) {
    //     cipo_pin = _config.cipo_pin;
    //     copi_pin = _config.copi_pin;
    //     sck_pin = _config.sck_pin;
    //     cs_pin = _config.cs_pin;
    //     int_pin = _config.int_pin;
    // };
};

struct icm42670_mmc5983_config {
    mems_config imu_config;
    mems_config mag_config;

    // TODO: Set default CS pin and magnetometer address
    icm42670_mmc5983_config(mems_config &_imu_config, mems_config &_mag_config) : imu_config(_imu_config), mag_config(_mag_config) {};
};

class ICM42670_MMC5983_IMU : public IMU<icm42670_mmc5983_config> {
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

        // Configuration parameters
        bool imu_use_interrupts = false;
        bool mag_use_interrupts = false;

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

    ICM42670_MMC5983_IMU() {
        lastTime = micros();
    }
    void magnetometerCalibration();
};
#endif