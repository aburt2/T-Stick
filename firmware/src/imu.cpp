#include "imu.h"

bool IMU::initialise(uint8_t I2C_ADDR) {
    Wire.begin();

    // [enabled] turns the gyro on or off.
    imuSensor.settings.gyro.enabled = true;
    // [scale] sets the full-scale range of the gyroscope.
    // scale can be set to either 245, 500, or 2000 dps
    // Travis West 2022-11-02: I was able to saturate the output with 2000 dps, so this seems like an appropriate setting.
    imuSensor.settings.gyro.scale = 2000;
    // [sampleRate] sets the output data rate (ODR) of the gyro
    // sampleRate can be set between 1-6
    // 1 = 14.9    4 = 238
    // 2 = 59.5    5 = 476
    // 3 = 119     6 = 952
    imuSensor.settings.gyro.sampleRate = 3; // 59.5Hz ODR
    // [bandwidth] can set the cutoff frequency of the gyro.
    // Allowed values: 0-3. Actual value of cutoff frequency
    // depends on the sample rate. (Datasheet section 7.12)
    imuSensor.settings.gyro.bandwidth = 0;
    // [lowPowerEnable] turns low-power mode on or off.
    imuSensor.settings.gyro.lowPowerEnable = false; // LP mode off
    // [HPFEnable] enables or disables the high-pass filter
    imuSensor.settings.gyro.HPFEnable = true; // HPF disabled
    // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
    // Allowable values are 0-9. Value depends on ODR.
    // (Datasheet section 7.14)
    imuSensor.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
    // [flipX], [flipY], and [flipZ] are booleans that can
    // automatically switch the positive/negative orientation
    // of the three gyro axes.
    imuSensor.settings.gyro.flipX = false; // Don't flip X
    imuSensor.settings.gyro.flipY = false; // Don't flip Y
    imuSensor.settings.gyro.flipZ = false; // Don't flip Z
    // [enabled] turns the acclerometer on or off.
    imuSensor.settings.accel.enabled = true; // Enable accelerometer
    // [enableX], [enableY], and [enableZ] can turn on or off
    // select axes of the acclerometer.
    imuSensor.settings.accel.enableX = true; // Enable X
    imuSensor.settings.accel.enableY = true; // Enable Y
    imuSensor.settings.accel.enableZ = true; // Enable Z
    // [scale] sets the full-scale range of the accelerometer.
    // accel scale can be 2, 4, 8, or 16 g's
    // Travis West 2022-11-02: In my experiments I found that the effort required
    // to get much more than 7.5 g of acceleration was significant enough that I
    // was worried about causing damage the internal wiring of the instrument.
    // As such, I think 8 g full scale range or less is appropriate, at least
    // until such time as the mechanical robustness of the instrument is improved.
    imuSensor.settings.accel.scale = 8;
    // [sampleRate] sets the output data rate (ODR) of the
    // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
    // DISABLED! Otherwise accel sample rate = gyro sample rate.
    // accel sample rate can be 1-6
    // 1 = 10 Hz    4 = 238 Hz
    // 2 = 50 Hz    5 = 476 Hz
    // 3 = 119 Hz   6 = 952 Hz
    imuSensor.settings.accel.sampleRate = 3;
    // [bandwidth] sets the anti-aliasing filter bandwidth.
    // Accel cutoff frequency can be any value between -1 - 3. 
    // -1 = bandwidth determined by sample rate
    // 0 = 408 Hz   2 = 105 Hz
    // 1 = 211 Hz   3 = 50 Hz
    imuSensor.settings.accel.bandwidth = 0; // BW = 408Hz
    // [highResEnable] enables or disables high resolution 
    // mode for the acclerometer.
    imuSensor.settings.accel.highResEnable = false; // Disable HR
    // [highResBandwidth] sets the LP cutoff frequency of
    // the accelerometer if it's in high-res mode.
    // can be any value between 0-3
    // LP cutoff is set to a factor of sample rate
    // 0 = ODR/50    2 = ODR/9
    // 1 = ODR/100   3 = ODR/400
    imuSensor.settings.accel.highResBandwidth = 0;  
    // [enabled] turns the magnetometer on or off.
    imuSensor.settings.mag.enabled = true; // Enable magnetometer
    // [scale] sets the full-scale range of the magnetometer
    // mag scale can be 4, 8, 12, or 16 Gs
    // Travis West 2022-11-02: Considering that the Earth's magnetic field is
    // generally less than 1 Gs, the lowest setting available is likely best.
    // A higher setting could be used if the sensor were installed next to a
    // strong magnetic field, such as a magnet or speaker, since then the reading
    // would not be saturated and the bias from the magnet could potentially be
    // removed.
    imuSensor.settings.mag.scale = 4;
    // [sampleRate] sets the output data rate (ODR) of the
    // magnetometer.
    // mag data rate can be 0-7:
    // 0 = 0.625 Hz  4 = 10 Hz
    // 1 = 1.25 Hz   5 = 20 Hz
    // 2 = 2.5 Hz    6 = 40 Hz
    // 3 = 5 Hz      7 = 80 Hz
    imuSensor.settings.mag.sampleRate = 5; // Set OD rate to 20Hz
    // [tempCompensationEnable] enables or disables 
    // temperature compensation of the magnetometer.
    imuSensor.settings.mag.tempCompensationEnable = false;
    // [XYPerformance] sets the x and y-axis performance of the
    // magnetometer to either:
    // 0 = Low power mode      2 = high performance
    // 1 = medium performance  3 = ultra-high performance
    imuSensor.settings.mag.XYPerformance = 3; // Ultra-high perform.
    // [ZPerformance] does the same thing, but only for the z
    imuSensor.settings.mag.ZPerformance = 3; // Ultra-high perform.
    // [lowPowerEnable] enables or disables low power mode in
    // the magnetometer.
    imuSensor.settings.mag.lowPowerEnable = false;
    // [operatingMode] sets the operating mode of the
    // magnetometer. operatingMode can be 0-2:
    // 0 = continuous conversion
    // 1 = single-conversion
    // 2 = power down
    imuSensor.settings.mag.operatingMode = 0; // Continuous mode

    imuSensor.begin();
    return 1;
}

bool IMU::getSensorData() {
    // read IMU and update puara-gestures
    if (imuSensor.accelAvailable()) {
        imuSensor.readAccel();
        // Save data
        ax = imuSensor.calcAccel(imuSensor.ax);
        ay = imuSensor.calcAccel(imuSensor.ay);
        az = imuSensor.calcAccel(imuSensor.az);
    }
    if (imuSensor.gyroAvailable()) {
        imuSensor.readGyro();
        // Save data
        gx = imuSensor.calcGyro(imuSensor.gx);
        gy = imuSensor.calcGyro(imuSensor.gy);
        gz = imuSensor.calcGyro(imuSensor.gz);
    }
    if (imuSensor.magAvailable()) {
        imuSensor.readMag();
        // Save data
        mx = imuSensor.calcMag(imuSensor.mx);
        my = imuSensor.calcMag(imuSensor.my);
        mz = imuSensor.calcMag(imuSensor.mz);
    }
    return 1;
}