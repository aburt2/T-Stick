// Virtual class for fuel gauge
#ifndef _TSTICK_FUELGAUGE_H_
#define _TSTICK_FUELGAUGE_H_

#include <Arduino.h>
#include <vector>


template<typename FUELGAUGE_CONFIG>
class FUELGAUGE
{
    public:
        // methods
        // Initialise Fuel Gauge
        typedef FUELGAUGE_CONFIG batt_config;
        virtual bool init(batt_config config, bool reset = false);

        // Get Battery Data (analog meausrements + modelguage outputs)
        virtual void getBatteryData();
};

#endif