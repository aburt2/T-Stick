#include <sensor.h>

bool sensor::init(int sensoraddress, int extraParam){
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