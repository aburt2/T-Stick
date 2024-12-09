#include "icm42670_mmc5633_imu.h"

bool ICM42670_MMC5633_IMU::initIMU(imu_config config) {
    // TODO: Write setup routine for the imu
    
    return true;
}

void ICM42670_MMC5633_IMU::getData() {
    //TODO: write method to get data from the imu + magnetometer
    // Get data from the IMU and save it
    // accl: x,y,z
    // gyro: x,y,z
    // mag: x,y,z
}

void ICM42670_MMC5633_IMU::sleep() {
    // TODO: write method to put the imu + mag into deep sleep
    // make the imu (accel, gyro and magnetometer go to sleep)
}

void ICM42670_MMC5633_IMU::clearInterrupt() {
    // TODO: write method to clear interrupts
    // clear any interrupts
}