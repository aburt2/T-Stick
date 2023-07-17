
#ifndef FUELGAUGE2_H
#define FUELGAUGE2_H

#include <Arduino.h>
#include <Adafruit_LC709203F.h>
#include <sensor.h>

class FuelGauge2 : public sensor{
    public:
        bool initialise(uint8_t I2C_ADDR = 0x30);
        bool getSensorData();
        int getData(int data_index);
        void getChargeRate();
        Adafruit_LC709203F maxlipo;      // for fual gauge
        float percentage;
        float voltage;
        float chargerate;
};

#endif