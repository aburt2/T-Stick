// Include Bela Trill Libraries
// Touch library for the trill craft board from Bela


#ifndef TRILLTOUCH_H
#define TRILLTOUCH_H

#include <Arduino.h>
#include <Trill.h>
#include <vector>
#include <touch.h>

#define TRILL_BASETOUCHSIZE 30

struct trill_config {
    Trill::Device touchdevice; // what device is used for touch sensing
    int touchsize; // Size of touch sensor array
    int touch_threshold; // threshold to detect touch
    int touch_mode; // mode for processing touch data
    int comm_mode; // communication mode for the touch sensor
};

class TrillTouch: public Touch<trill_config> {
    public:
        uint8_t initTouch(touch_config trill_config);
        void readTouch();
        int getData(int data_index);
        
        // trill board properties
        uint8_t craft_i2c_addr = 0x30;
        uint8_t flex_i2c_addr = 0x48;
        Trill trillSensor1;      // for Trill Craft
        Trill trillSensor2;      // for Trill Craft
        Trill trillSensor3;      // for Trill Craft
        Trill trillSensor4;      // for Trill Craft
        std::vector<Trill *> touchArray = {&trillSensor1, &trillSensor2, &trillSensor3, &trillSensor4}; // array of touch data

        int data[120];
        byte touchStatus = 0;
        int newData = 0;
        int touch[120];          // /instrument/touch/touch, i..., 0 or 1, ... (1 per stripe)
        float normTouch[120];    // /instrument/touch/norm, i..., 0--1, ... (1 per stripe)
        int discreteTouch[120];    // /instrument/touch/raw, i..., 0--1, ... (1 per stripe)
        void cookData();
        float num_boards = 1; // number of touch boards, set half numbers to indicate if using only half of the touch data
        int touchSize = 30;
        
        // Running or not
        bool running = true;
    private:
        int maxTouchValue = 50;
};

#endif