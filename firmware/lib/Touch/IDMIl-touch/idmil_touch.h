/* IDMIL capsense library

Edu Meneses - IDMIL, McGill University (2022)

Based on Alex Nieva's capsense Arduino code (IDMIL, 2019)

*/

#ifndef CAPSENSE_H
#define CAPSENSE_H

//#include <stdint.h>
#include <Wire.h>
#include <SPI.h>
#include <touch.h>

struct idmil_touch_config {
    int touchdevice; // what device is used for touch sensing
    int touchsize; // Size of touch sensor array
    int touch_threshold; // threshold to detect touch
    int touch_mode; // mode for processing touch data
    int comm_mode; // communication mode for the touch sensor
};

class Capsense: public Touch<idmil_touch_config> {
    public:
        void initCapsense(uint8_t I2C_ADDR);
        uint8_t initTouch(touch_config idmilTouch_config);
        void readTouch();
        void cookData();
        int getData(int data_index);
        int newData = 0;
        int data[8];
        int touch[64]; // /raw/capsense, i..., 0--255, ... (1 int per 8 capacitive stripes -- 8 bits) - originaly RawData.touch
        int discreteTouch[64];
        uint8_t touchStripsSize;
        int touchSize;
    private:
        void capsenseRequest(uint8_t address,uint8_t request, uint8_t answer_size);
        void reorderCapsense (int *origArray, uint8_t arraySize);
        int bitReadLeftToRight (int number, int position, int type_size = 8);
        int bitReadRightToLeft (int number, int position);
        uint8_t answer1;
        uint8_t answer2;
        uint8_t capsense_addresses[4]; // max 4 capsenses
        uint8_t nCapsenses = 0;
};

#endif
