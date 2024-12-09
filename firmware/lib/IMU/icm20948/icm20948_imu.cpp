#include "icm20948_imu.h"

bool ICM20948_IMU::initIMU(imu_config config) {
    // Initialise IMU based on Sparkfun IMC20948 library Advanced Example
    // https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/examples/Arduino/Example2_Advanced/Example2_Advanced.ino
    icm20948_imu.begin(config._i2c,config.adr_val);

    // Reset IMU so it is in known stable
    icm20948_imu.swReset();
    delay(200);

    // wake up IMU
    icm20948_imu.sleep(false);
    icm20948_imu.lowPower(false);

    // set continuous sample mode
    icm20948_imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);

    // Set scale settings
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
    myFSS.a = gpm8; 
    myFSS.g = dps2000;
    icm20948_imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                    // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                    // acc_d111bw4_n136bw
                                    // acc_d50bw4_n68bw8
                                    // acc_d23bw9_n34bw4
                                    // acc_d11bw5_n17bw
                                    // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                    // acc_d473bw_n499bw
    myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5
    icm20948_imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    ICM_20948_Status_e accDLPEnableStat = icm20948_imu.enableDLPF(ICM_20948_Internal_Acc, false);
    ICM_20948_Status_e gyrDLPEnableStat = icm20948_imu.enableDLPF(ICM_20948_Internal_Gyr, false);

    // Enable magnetometer
    icm20948_imu.startupMagnetometer();

    // Enable and setup interrupts
    icm20948_imu.cfgIntActiveLow(true);
    icm20948_imu.cfgIntOpenDrain(false);
    icm20948_imu.cfgIntLatch(false);
    icm20948_imu.intEnableRawDataReady(true);
    return true;
}

void ICM20948_IMU::getData() {
    // Get data from the IMU and save it
    icm20948_imu.getAGMT();

    // read icm20948_imu
    accl[0] = -icm20948_imu.accX() / 1000;
    accl[1] = -icm20948_imu.accY() / 1000;
    accl[2] = icm20948_imu.accZ() / 1000;

    // in degrees/sc
    gyro[0] = icm20948_imu.gyrX();
    gyro[1] = icm20948_imu.gyrY();
    gyro[2] = icm20948_imu.gyrZ();

    // uTesla
    magn[0] = icm20948_imu.magX();
    magn[1] = icm20948_imu.magY();
    magn[2] = icm20948_imu.magZ();
}

void ICM20948_IMU::sleep() {
    icm20948_imu.sleep(true);
}

void ICM20948_IMU::clearInterrupt() {
    icm20948_imu.clearInterrupts();
}