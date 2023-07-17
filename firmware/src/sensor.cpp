#include <sensor.h>

// Define static members
std::string sensor::name;
std::string sensor::type;
std::string sensor::comms;
uint8_t sensor::address;

bool sensor::init(uint8_t sensoraddress, uint8_t extraParam){
    return initialise(sensoraddress, extraParam);
}

bool sensor::readData() {
    return getSensorData();
}

bool sensor::getSensorData() {
    return 1;
}

bool sensor::initialise(uint8_t sensoraddress, uint8_t extraParam) {
    return 1;
}