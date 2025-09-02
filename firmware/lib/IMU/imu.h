#ifndef _TSTICK_IMU_H
#define _TSTICK_IMU_H

template<typename IMU_CONFIG>
class IMU {
    public:
        // Methods
        typedef IMU_CONFIG imu_config;
        virtual bool initIMU(imu_config config) = 0;
        
        // Read data
        virtual void getData() = 0;
        virtual void sleep() = 0;
        virtual void clearInterrupt() = 0;

        // Store data
        float accl[3];
        float gyro[3];
        float magn[3];
};
#endif