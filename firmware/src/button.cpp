
// Button (or button) function

#include "button.h"
bool sensor::init(uint8_t sensoraddress, uint8_t extraParam) {
    return initialise(sensoraddress, extraParam);
}

bool sensor::readData() {
    return getSensorData();
}

bool Button::getSensorData() {
    buttonState = !digitalRead(pin);
    if (buttonState) {
        if (!Button::button) {
            Button::button = true;
            Button::timer = millis();
        }
        if (millis() - Button::timer > Button::holdInterval) {
            Button::hold = true;
        }
    }
    else if (Button::hold) {
        Button::hold = false;
        Button::button = false;
    }
    else {
        if (Button::button) {
            Button::button = false;
            Button::pressTime = millis() - Button::timer;
            Button::timer = millis();
        }
    }
    return 1;
}

bool Button::initialise(int &buttonPin) {
    Button::pin = buttonPin;
    pinMode(Button::pin, INPUT_PULLUP);
    return 1;
}

bool Button::getButton() {
    return Button::button;
}

unsigned int Button::getState() {
    return Button::buttonState;
};

unsigned int Button::getPressTime() {
    return Button::pressTime;
}

bool Button::getHold() {
    return Button::hold;
}

unsigned int Button::getHoldInterval() {
    return Button::holdInterval;
}

unsigned int Button::setHoldInterval(int value) {
    Button::holdInterval = value;
    return 1;
}

