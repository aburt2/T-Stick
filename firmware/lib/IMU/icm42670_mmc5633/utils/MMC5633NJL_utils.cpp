#include "MMC5633NJL_utils.h"
#include <Arduino.h>

void setupMagnetometer(Adafruit_MMC5603 &mag) {

  if (!mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {  // I2C mode
    //There was a problem detecting the MMC5603
    Serial.println("Ooops, no MMC5603 detected");
    while (1) delay(10);
  }
}

sensors_event_t readMagnetometer(Adafruit_MMC5603 &mag, bool serial_print) {

  // obtain magnetometer event data
  sensors_event_t event;
  mag.getEvent(&event);

  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / PI;

  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }

  if (serial_print) {
    Serial.print("Compass Heading: ");
    Serial.print(heading);
    Serial.println("  Deg");
    Serial.print("\n");
    delay(500);
  }

  return event;
}
