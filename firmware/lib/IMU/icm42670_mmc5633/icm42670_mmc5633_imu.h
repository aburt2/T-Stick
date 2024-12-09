#ifndef _ICM4670_MMC5633_IMU_H_
#define _ICM4670_MMC5633_IMU_H_

#include "imu.h"
#include "Wire.h" // needed for magnetometer
#include "SPI.h"  // will probably be needed (might use ESP32 hal library directly)

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
};
#endif