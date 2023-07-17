// include fuel gauge header
#include <fuelgauge2.h>

bool FuelGauge2::initialise(uint8_t I2C_ADDR) {
    if (!maxlipo.begin()) {
        // if initialisation fails return 0
        return 0;
    }
    // The alert pin can be used to detect when the voltage of the battery goes below or
    // above a voltage, you can also query the alert in the loop.
    maxlipo.setAlarmVoltage(2.0);
    return maxlipo.setPackSize(LC709203F_APA_3000MAH);
}

bool FuelGauge2::getSensorData() {
    // Get the cell and battery voltage
    percentage = maxlipo.cellPercent();
    voltage = maxlipo.cellVoltage();

    // Get charge rate
    FuelGauge2::getChargeRate();

    // return 21 if successful
    return 1;
}

void FuelGauge2::getChargeRate() {
    // Get the charge rate of the battery
    chargerate = 999; //Charge rate not measured by LC709203F
}