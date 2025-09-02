#include "icm42670_mmc5983_imu.h"
#include "SensorFusion.h"
#include <float.h>

// Instantiate an ICM42670 with LSB address set to 0 and 


// Create instance of magnetometer
SFE_MMC5983MA ICM_MAG;

// Setup spi settings
SPISettings mag_spi_settings(10000000, MSBFIRST, SPI_MODE3);

// Instance of SensorFusion class
SF SensorFusion;

// error callback for potatos
void mag_error(SF_MMC5983MA_ERROR errorCode) {
    Serial.println(ICM_MAG.errorCodeString(errorCode));
}

// Interrupt routines
volatile bool newMagData = false;
volatile bool newIMUData = false;
void imu_interrupt();
void mag_interrupt();

void imu_interrupt()
{
    newIMUData = true;
}
void mag_interrupt()
{
    newMagData = true;
}

// SPIn Classes
SPIClass imu_spi(FSPI);
SPIClass mag_spi(HSPI);

// clock speed 40MHz
ICM42670 ICM_IMU(imu_spi, GPIO_NUM_10, 24000000);

bool ICM42670_MMC5983_IMU::initIMU(imu_config config) {
    // Start SPI with custom pins
    imu_spi.begin(config.imu_config.sck_pin, config.imu_config.cipo_pin, config.imu_config.copi_pin, config.imu_config.cs_pin);
    while (mag_spi.begin(config.mag_config.sck_pin, config.mag_config.cipo_pin, config.mag_config.copi_pin, config.mag_config.cs_pin) == false) {
        Serial.println("Magnetometer SPI bus failed to init. Retrying...");
        delay(1000);
    };

    // Set pins as outputs
    pinMode(imu_spi.pinSS(), OUTPUT);  //VSPI SS
    pinMode(mag_spi.pinSS(), OUTPUT);  //HSPI SS


    // Setup 6DOF IMU
    ICM_IMU.spi_cs = config.imu_config.cs_pin; // update CS pin
    while (ICM_IMU.begin() != 0) {
        Serial.println("ICM42670 did not respond. Retrying...");
        Serial.printf("CS PIN: GPIO %d\n", imu_spi.pinSS());
        delay(1000);
    };
    ICM_IMU.startAccel(1600, 16);  // Set ODR and range for accelerometer
    ICM_IMU.startGyro(1600, 2000);  // Set ODR and range for gyroscope
    delay(100); // Wait for IMU to stabilise
    
    // Setup error callback
    ICM_MAG.setErrorCallback(mag_error);
    // while (ICM_MAG.begin(mag_spi.pinSS(), mag_spi_settings, mag_spi) == false) {
    //     delay(500);
    //     ICM_MAG.isConnected();
    //     if (!ICM_MAG.softReset()) {
    //         Serial.println("magnetometer did not reset...");
    //     } else {
    //         Serial.println("successful reset...");
    //     }
    //     delay(500);
    // }
    ICM_MAG.begin(mag_spi.pinSS(), mag_spi_settings, mag_spi);

    // Setup magnetometer
    ICM_MAG.softReset();
    ICM_MAG.setFilterBandwidth(800);
    ICM_MAG.setContinuousModeFrequency(1000);
    ICM_MAG.enableAutomaticSetReset();
    ICM_MAG.enableContinuousMode();
    ICM_MAG.enableInterrupt();
    newMagData = true;

    // Setu
    return true;
}

void ICM42670_MMC5983_IMU::getData() {
    //TODO: write method to get data from the imu + magnetometer
    // Get data from the IMU and save it
    // accl: x,y,z
    // gyro: x,y,z
    // mag: x,y,z

    // Read raw imu data
    inv_imu_sensor_event_t imu_event;
    ICM_IMU.getDataFromRegisters(imu_event);

    // Read raw magnetometer
    uint32_t rawMagX, rawMagY, rawMagZ;
    ICM_MAG.readFieldsXYZ(&rawMagX, &rawMagY, &rawMagZ);

    // Save data to class
    accl[0] = -((float)imu_event.accel[0] * accelSensitivity);
    accl[1] = -((float)imu_event.accel[1] * accelSensitivity);
    accl[2] = ((float)imu_event.accel[2] * accelSensitivity);
    
    gyro[0] = (float)imu_event.gyro[0] * gyroMultipier;
    gyro[1] = (float)imu_event.gyro[1] * gyroMultipier;
    gyro[2] = (float)imu_event.gyro[2] * gyroMultipier;

    // Magnetometer
    // Save magnetometer data to class
    magn[0] = -rawMagX;
    magn[1] = -rawMagY;
    magn[2] = rawMagZ;
}

void ICM42670_MMC5983_IMU::updateOrientation() {
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

void ICM42670_MMC5983_IMU::magnetometerCalibration() {

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


void ICM42670_MMC5983_IMU::sleep() {
    // TODO: write method to put the imu + mag into deep sleep
    // make the imu (accel, gyro and magnetometer go to sleep)

    /************************************\
    Accelerometer and gyroscope to sleep*/
    // Value to configure accl and gyro to sleep
    uint8_t sleepConfig = 0x00;
    uint8_t imuSleepReg = 0x4E;
    
    // // Put CS pin low in order to communicate with the IMU
    // digitalWrite(ICM_IMU.spi_cs, LOW);
    

    // // 0x7f is the mask to write to the register
    // SPI.transfer(imuSleepReg & 0x7F);
    // SPI.transfer(sleepConfig);
    // digitalWrite(ICM_IMU.spi_cs, HIGH);

    /***********************\
    /*Magnetometer to sleep*/
    // TODO
}

void ICM42670_MMC5983_IMU::clearInterrupt() {
    // TODO: write method to clear interrupts
    // clear any interrupts
    
}