// include fuel gauge header
#include <fuelgauge.h>

bool FuelGauge::initialise(uint8_t I2C_ADDR) {
    // Based on the advanced usage from https://learn.adafruit.com/adafruit-max17048-lipoly-liion-fuel-gauge-and-battery-monitor/arduino
    if (!maxlipo.begin()) {
        // if initialisation fails return 0
        return false;
    }

    // Set voltage
    maxlipo.setResetVoltage(2.5);

    // Hibernation mode reduces how often the ADC is read, for power reduction. There is an automatic
    // enter/exit mode but you can also customize the activity threshold both as voltage and charge rate
    maxlipo.setActivityThreshold(0.15);
    maxlipo.setHibernationThreshold(5);

    // The alert pin can be used to detect when the voltage of the battery goes below or
    // above a voltage, you can also query the alert in the loop.
    maxlipo.setAlertVoltages(2.0, 4.2);

    // Return 1 if everything went well
    return true;
}

bool FuelGauge::getSensorData() {
    // Get the cell and battery voltage
    percentage = maxlipo.cellPercent();
    voltage = maxlipo.cellVoltage();

    // Get the charge rate
    FuelGauge::getChargeRate();

    // Return 1 for success
    return 1;
}

void FuelGauge::getChargeRate() {
    // Get the charge rate of the battery
    chargerate = maxlipo.chargeRate();
}