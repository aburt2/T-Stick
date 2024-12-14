#include "IMU_utils.h"

void setupIMU(ICM42670 &imu) {
    // Initialise IMU
    int ret = imu.begin();
    if (ret != 0) {
        Serial.println("ICM42670 initialization failed");
        while (1);  // Stop execution if initialization fails
    }
    imu.startAccel(1000, 16);  // Set ODR and range for accelerometer
    imu.startGyro(1000, 2000);  // Set ODR and range for gyroscope
    delay(100);  // Wait for IMU to stabilize
}

inv_imu_sensor_event_t readIMU(ICM42670 &imu, bool serial_print) {

    // Obtain IMU event data
    inv_imu_sensor_event_t imu_event;
    imu.getDataFromRegisters(imu_event);
    
    float G = 9.80665;
    
    // Max output data range (ODR) is set to max (+/-16G),
    // corresponding sensitivity is 2048. 
    // To convert raw data (g) to m/sec^2, divide by sensitivity
    // as specified by ODR and multiply by acceleration due to
    // gravity (9.80665m/sec^2)
    float accelSensitivity = 2048.00;
    float gyroSensitivity = 16.40;

    // Print processed accelerometer data
    if (serial_print) {
        Serial.print("AccelX: ");
        Serial.print(((float)imu_event.accel[0] / accelSensitivity) * G);
        Serial.println("  m/sec^2");
        Serial.print("AccelY: ");
        Serial.print(((float)imu_event.accel[1] / accelSensitivity) * G);
        Serial.println("  m/sec^2");
        Serial.print("AccelZ: ");
        Serial.print(((float)imu_event.accel[2] / accelSensitivity) * G);
        Serial.println("  m/sec^2");
        Serial.print("\n");

        // ODR of +/- 2000deg/s needs to be converted to rad/sec
        // so multiply by PI, and dividy by 180 to convert deg to rad
        // and than divide by gyro sensitivity (16.4 at selected ODR)

        // Print processed gyroscope data
        Serial.print("GyroX: ");
        Serial.print(((float)imu_event.gyro[0]) * PI / (gyroSensitivity * 180.0));
        Serial.println("  rad/s");
        Serial.print("GyroY: ");
        Serial.print(((float)imu_event.gyro[1]) * PI / (gyroSensitivity * 180.0));
        Serial.println("  rad/s");
        Serial.print("GyroZ: ");
        Serial.print(((float)imu_event.gyro[2]) * PI / (gyroSensitivity * 180.0));
        Serial.println("  rad/s");
        Serial.print("Temperature:");
        Serial.print((imu_event.temperature)/128+25);
        Serial.println(" C\n");
        Serial.print("\n");
    }
    return imu_event;
}
