#ifndef NPM1300_ESP32_PWR_CTR_H
#define NPM1300_ESP32_PWR_CTR_H

#include <Arduino.h>
#include <Wire.h>
#include <iostream>

#include <npm1300.hpp>


class npmx_pmic {
    public:
        NPM1300_PMIC *_npmx = NULL;
        npmx_pmic(NPM1300_PMIC &_npmx_class);

        void begin();
        void begin(NPM1300_PMIC &_npmx_class);
        void set_power_limits();
        void sleep();
    private:

};
#endif

