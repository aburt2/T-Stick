// FSR function

#include "fsr.h"
bool sensor::init(uint8_t sensoraddress, uint8_t extraParam = 0) {
    return initialise(sensoraddress, extraParam);
}

bool sensor::readData() {
    return getSensorData();
}

bool Fsr::initialise(int &fsr_pin, int offsetValue){
    Fsr::pin = fsr_pin;
    Fsr::offset = offsetValue;
    return 1;
}

bool Fsr::getSensorData() {
    Fsr::value = analogRead(Fsr::pin);
    Fsr::normValue = constrain((Fsr::value - Fsr::offset) / (4095 - Fsr::offset), 0, 1);
    Fsr::cookedValue = Fsr::value - Fsr::offset;
    return 1;
}

float Fsr::getValue() {
    return Fsr::value;
}

float Fsr::getNormValue() {
    return Fsr::normValue;
}

float Fsr::getCookedValue() {
    return Fsr::cookedValue;
}

int Fsr::getOffset(){
    return Fsr::offset;
}

int Fsr::setOffset(int offsetValue){
    Fsr::offset = offsetValue;
    return 1;
}