// Create library for managing sensors

#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <Wire.h>

class sensor {
    public:
        bool init(int sensoraddress, int extraParam = 0);
        bool readData();
        virtual bool initialise(uint8_t sensoraddress, uint8_t extraParam = 0);
        virtual bool getSensorData();
    private:
        static std::string name;
        static std::string type;
        static std::string comms;
        static uint8_t address;
};

#endif