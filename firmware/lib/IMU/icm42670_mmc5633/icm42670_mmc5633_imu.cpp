#include "icm42670_mmc5633_imu.h"
#include "SensorFusion.h"
#include <float.h>

// Instantiate an ICM42670 with LSB address set to 0 and 
// clock speed 40MHz
ICM42670 ICM_IMU(SPI, GPIO_NUM_10, 40000000);

// Assign a unique ID to this sensor
Adafruit_MMC5603 mag = Adafruit_MMC5603(MAG_ID);

// Instance of SensorFusion class
SF SensorFusion;

bool ICM42670_MMC5633_IMU::initIMU(imu_config config) {
    // TODO: Write setup routine for the imu
    ICM_IMU.spi = &config._spi;
    ICM_IMU.spi_cs = config.ag_cs_pin;

    // Setup 6DOF IMU
    setupIMU(ICM_IMU);

    // Setup magnetometer
    mag.begin(MMC56X3_DEFAULT_ADDRESS, &config._i2c);
    mag.setDataRate(250);
    mag.setContinuousMode(true);
    return true;
}

void ICM42670_MMC5633_IMU::getData() {
    //TODO: write method to get data from the imu + magnetometer
    // Get data from the IMU and save it
    // accl: x,y,z
    // gyro: x,y,z
    // mag: x,y,z
    inv_imu_sensor_event_t imu_event;
    sensors_event_t mag_event;
    imu_event = readIMU(ICM_IMU, false);
    mag_event = readMagnetometer(mag, false);

    // Save data to class
    accl[0] = -((float)imu_event.accel[0] * accelSensitivity);
    accl[1] = ((float)imu_event.accel[1] * accelSensitivity);
    accl[2] = ((float)imu_event.accel[2] * accelSensitivity);
    
    gyro[0] = (float)imu_event.gyro[0] * gyroMultipier;
    gyro[1] = (float)imu_event.gyro[1] * gyroMultipier;
    gyro[2] = (float)imu_event.gyro[2] * gyroMultipier;

    // Magnetometer
    // Save magnetometer data to class
    magn[0] = mag_event.magnetic.x;
    magn[1] = mag_event.magnetic.y;
    magn[2] = mag_event.magnetic.z;
}

void ICM42670_MMC5633_IMU::updateOrientation() {
    unsigned long now = micros();
    deltaT = (now - lastTime)/1000000.0f;
    lastTime = now;

    getData();

    SensorFusion.MadgwickUpdate(
        gyro[0], gyro[1], gyro[2],
        accl[0], accl[1], accl[2],
        magn[0], magn[1], magn[2], 
        deltaT
    );
    
    yaw = SensorFusion.getYaw();
    roll = SensorFusion.getRoll();
    pitch = SensorFusion.getPitch();
    
}

void ICM42670_MMC5633_IMU::magnetometerCalibration() {

    float magnMin_x, magnMax_x;
    float magnMin_y, magnMax_y;
    float magnMin_z, magnMax_z;

    float magnBias_x, magnBias_y, magnBias_z = 0;

    long lastDisplaytime = millis();

    int counter = 0;

    for (int i = 0; i < 100; i++) {
        getData();
        magnMin_x = magnMax_x = magn[0];
        magnMin_y = magnMax_y = magn[1];
        magnMin_z = magnMax_z = magn[2];    
    }

    //sleep();

    while(1) {
        getData();

        if (counter > 5) {
            magnBias_x = (magnMax_x + magnMin_x) / 2.0f;
            magnBias_y = (magnMax_y + magnMin_y) / 2.0f;
            magnBias_z = (magnMax_z + magnMin_z) / 2.0f;
        }

        if (magn[0] < magnMin_x) magnMin_x = magn[0];
        if (magn[0] > magnMax_x) magnMax_x = magn[0];

        if (magn[1] < magnMin_y) magnMin_y = magn[1];
        if (magn[1] > magnMax_y) magnMax_y = magn[1];

        if (magn[2] < magnMin_z) magnMin_z = magn[2];
        if (magn[2] > magnMax_z) magnMax_z = magn[2];

        if((millis() - lastDisplaytime) > 1000) {
            while(!Serial) {
                delay(10);
            }
            Serial.print("Mag Minimums: "); Serial.print(magnMin_x); Serial.print("  ");Serial.print(magnMin_y); Serial.print("  "); Serial.print(magnMin_z); Serial.println();
            Serial.print("Mag Maximums: "); Serial.print(magnMax_x); Serial.print("  ");Serial.print(magnMax_y); Serial.print("  "); Serial.print(magnMax_z);
            Serial.println(); 
            Serial.print("Raw Magn X: "); Serial.println(magn[0]);
            Serial.print("Raw Magn Y: "); Serial.println(magn[1]);
            Serial.print("Raw Magn Z: "); Serial.println(magn[2]);
            Serial.println();
            Serial.print("X Bias: "); Serial.println(magnBias_x);
            Serial.print("Y Bias: "); Serial.println(magnBias_y);
            Serial.print("Z Bias: "); Serial.println(magnBias_z);
            Serial.println();
            lastDisplaytime = millis(); 

            counter++;

        }
    }

}


void ICM42670_MMC5633_IMU::sleep() {
    // TODO: write method to put the imu + mag into deep sleep
    // make the imu (accel, gyro and magnetometer go to sleep)

    /************************************\
    Accelerometer and gyroscope to sleep*/
    // Value to configure accl and gyro to sleep
    uint8_t sleepConfig = 0x00;
    uint8_t imuSleepReg = 0x4E;
    
    // Put CS pin low in order to communicate with the IMU
    digitalWrite(ICM_IMU.spi_cs, LOW);
    

    // 0x7f is the mask to write to the register
    SPI.transfer(imuSleepReg & 0x7F);
    SPI.transfer(sleepConfig);
    digitalWrite(ICM_IMU.spi_cs, HIGH);

    /***********************\
    /*Magnetometer to sleep*/

    mag.setDataRate(0);
    mag.setContinuousMode(false);
}

void ICM42670_MMC5633_IMU::clearInterrupt() {
    // TODO: write method to clear interrupts
    // clear any interrupts
}