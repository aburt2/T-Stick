#ifndef IMU_UTILS_H
#define IMU_UTILS_H

#include "ICM42670P.h"
#include <Arduino.h>  // For Serial
// Function declarations
void setupIMU(ICM42670 &imu);
inv_imu_sensor_event_t readIMU(ICM42670 &imu, bool serial_print = true);

#endif