#ifndef MAGNETOMETER_UTILS_H
#define MAGNETOMETER_UTILS_H

#include <Adafruit_MMC56x3.h>

// Function declarations
void setupMagnetometer(Adafruit_MMC5603 &mag);
sensors_event_t readMagnetometer(Adafruit_MMC5603 &mag, bool serial_print = true);

#endif
