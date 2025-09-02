#ifndef TSTICK_TOUCH_H
#define TSTICK_TOUCH_H

#include <Arduino.h>
#include <vector>

#define BASETOUCHSIZE 30

/**
 * Touch Class: Parent class for touch controller for the t-stick
 * 
 * @author Albert (12/2024)
 * 
 */
template<typename TOUCH_CONFIG>
class Touch {
    public:
        /**
         * Command data: Structure for the communicating with STM32
         * 
         * @author Albert (06=8/2024)
         * 
         * @param cmd     command
         * @param payload min voltage for source
         * 
         */
        typedef TOUCH_CONFIG touch_config;
        virtual uint8_t initTouch(touch_config touch_config) = 0;
        virtual void readTouch() = 0;
};

#endif