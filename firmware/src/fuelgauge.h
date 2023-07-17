
#ifndef FUELGAUGE_H
#define FUELGAUGE_H

#include <Arduino.h>
#include <Adafruit_MAX1704X.h>
#include <sensor.h>

class FuelGauge : public sensor{
    public:
        bool initialise(uint8_t I2C_ADDR = 0x30);
        bool getSensorData();
        int getData(int data_index);
        void getChargeRate();
        Adafruit_MAX17048 maxlipo;      // for fual gauge
        float percentage;
        float voltage;
        float chargerate;
};

#endif