// fsr

#ifndef FSR_H
#define FSR_H

#include <Arduino.h>
#include <sensor.h>

class Fsr : public sensor {
    private:
        int pin;
        float value = 0;
        float normValue = 0;
        float cookedValue = 0;
        int offset = 0;
    public:
        bool initialise(int &fsr_pin, int offsetValue);
        bool getSensorData();
        float getValue();
        float getNormValue();
        float getCookedValue();
        int getOffset();
        int setOffset(int offsetValue);
};

#endif