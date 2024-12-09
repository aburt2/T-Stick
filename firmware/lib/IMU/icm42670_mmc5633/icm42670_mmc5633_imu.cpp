#include "icm42670_mmc5633_imu.h"

bool ICM42670_MMC5633_IMU::initIMU(imu_config config) {
    // Setup IMU
    
    return true;
}

void ICM42670_MMC5633_IMU::getData() {
    // Get data from the IMU and save it
    // accl: x,y,z
    // gyro: x,y,z
    // mag: x,y,z
}

void ICM42670_MMC5633_IMU::sleep() {
    // make the imu (accel, gyro and magnetometer go to sleep)
}

void ICM42670_MMC5633_IMU::clearInterrupt() {
    // clear any interrupts
}