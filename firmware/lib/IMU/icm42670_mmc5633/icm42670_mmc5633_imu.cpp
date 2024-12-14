#include "icm42670_mmc5633_imu.h"

// Instantiate an ICM42670 with LSB address set to 0 and 
// clock speed 40MHz
ICM42670 ICM_IMU(SPI, GPIO_NUM_10, 40000000);

// Assign a unique ID to this sensor
Adafruit_MMC5603 mag = Adafruit_MMC5603(MAG_ID);

bool ICM42670_MMC5633_IMU::initIMU(imu_config config) {
    // TODO: Write setup routine for the imu
    ICM_IMU.spi = &config._spi;
    ICM_IMU.spi_cs = config.ag_cs_pin;

    // Setup 6DOF IMU
    setupIMU(ICM_IMU);

    // Setup magnetometer
    mag.begin(MMC56X3_DEFAULT_ADDRESS, &config._i2c);
    mag.setDataRate(1000);
    mag.setContinuousMode(true);
    return true;
}

void ICM42670_MMC5633_IMU::getData() {
    //TODO: write method to get data from the imu + magnetometer
    // Get data from the IMU and save it
    // accl: x,y,z
    // gyro: x,y,z
    // mag: x,y,z
    inv_imu_sensor_event_t imu_event;
    sensors_event_t mag_event;
    imu_event = readIMU(ICM_IMU, false);
    mag_event = readMagnetometer(mag, false);

    // Sensitivity
    float G = 9.80665;
    float accelSensitivity = 2048.00;
    float gyroSensitivity = 16.40;

    // Save data to class
    accl[0] = ((float)imu_event.accel[0] / accelSensitivity) * G;
    accl[1] = ((float)imu_event.accel[1] / accelSensitivity) * G;
    accl[2] = ((float)imu_event.accel[2] / accelSensitivity) * G;
    
    gyro[0] = ((float)imu_event.gyro[0]) * PI / (gyroSensitivity * 180.0);
    gyro[1] = ((float)imu_event.gyro[1]) * PI / (gyroSensitivity * 180.0);
    gyro[2] = ((float)imu_event.gyro[2]) * PI / (gyroSensitivity * 180.0);

    // Magnetometer
    // Save magnetometer data to class
    magn[0] = mag_event.magnetic.x;
    magn[1] = mag_event.magnetic.y;
    magn[2] = mag_event.magnetic.z;

    // Calculate the angle of the vector y,x
    heading = (atan2(mag_event.magnetic.y,mag_event.magnetic.x) * 180) / PI;

    // Normalize to 0-360
    if (heading < 0) {
        heading = 360 + heading;
    }
}

void ICM42670_MMC5633_IMU::sleep() {
    // TODO: write method to put the imu + mag into deep sleep
    // make the imu (accel, gyro and magnetometer go to sleep)
}

void ICM42670_MMC5633_IMU::clearInterrupt() {
    // TODO: write method to clear interrupts
    // clear any interrupts
}