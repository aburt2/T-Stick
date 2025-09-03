#include "icm42670_mmc5983_imu.h"
#include <float.h>

// SPIn Classes
SPIClass * imu_spi = NULL;
SPIClass * mag_spi = NULL;

// Create instance of magnetometer
SFE_MMC5983MA ICM_MAG;
// Setup spi settings
SPISettings mag_spi_settings(10000000, MSBFIRST, SPI_MODE3);
// error callback for potatos
void mag_error(SF_MMC5983MA_ERROR errorCode) {
    Serial.println(ICM_MAG.errorCodeString(errorCode));
    if (errorCode == SF_MMC5983MA_ERROR::INVALID_DEVICE) {
        Serial.printf("PRODUCT ID READ: %d\n", ICM_MAG.product_id);
    }
}

// Create instance of IMU
ICM42670 ICM_IMU(*imu_spi, GPIO_NUM_10, 24000000);
// imu interrupt callback
int16_t imu_accl[3] = {0,0,0};
int16_t imu_gyro[3] = {0,0,0};
void imu_cb(inv_imu_sensor_event_t *evt) {
    if(ICM_IMU.isAccelDataValid(evt)&&ICM_IMU.isGyroDataValid(evt)) {
        imu_accl[0] = evt->accel[0];
        imu_accl[1] = evt->accel[1];
        imu_accl[2] = evt->accel[2];
        
        imu_gyro[0] = evt->gyro[0];
        imu_gyro[1] = evt->gyro[1];
        imu_gyro[2] = evt->gyro[2];
    }
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

bool ICM42670_MMC5983_IMU::initIMU(imu_config config) {
    // Initialise new SPI classes
    imu_spi = new SPIClass(FSPI);
    mag_spi = new SPIClass(HSPI);

    // Start SPI with custom pins
    imu_spi->begin(config.imu_config.sck_pin, config.imu_config.cipo_pin, config.imu_config.copi_pin, config.imu_config.cs_pin);
    while(!mag_spi->begin(config.mag_config.sck_pin, config.mag_config.cipo_pin, config.mag_config.copi_pin, config.mag_config.cs_pin)) {
        Serial.println("SPI bus failed to start.");
    }
    mag_spi->setDataMode(SPI_MODE3);

    // Set SS pins as outputs
    pinMode(config.mag_config.cs_pin, OUTPUT);
    pinMode(config.imu_config.cs_pin, OUTPUT);
    digitalWrite(config.mag_config.cs_pin, HIGH);
    digitalWrite(config.mag_config.cs_pin, LOW);

    // Setup interrupt pin
    pinMode(config.imu_config.int_pin, INPUT);
    pinMode(config.mag_config.int_pin, INPUT);

    // Setup interrupts
    attachInterrupt(digitalPinToInterrupt(config.mag_config.int_pin), mag_interrupt, RISING);

    // Setup 6DOF IMU
    ICM_IMU.spi = imu_spi;
    ICM_IMU.spi_cs = config.imu_config.cs_pin; // update CS pin
    while (ICM_IMU.begin() != 0) {
        Serial.println("ICM42670 did not respond. Retrying...");
        Serial.printf("CS PIN: GPIO %d\n", imu_spi->pinSS());
        delay(1000);
    };
    ICM_IMU.enableFifoInterrupt(config.imu_config.int_pin, imu_interrupt, 10);
    ICM_IMU.startAccel(1600, 16);  // Set ODR and range for accelerometer
    ICM_IMU.startGyro(1600, 2000);  // Set ODR and range for gyroscope
    delay(100); // Wait for IMU to stabilise
    
    // Setup Magnetometer
    ICM_MAG.mmc_io._spiPort = mag_spi;
    ICM_MAG.setErrorCallback(mag_error);
    // while (ICM_MAG.begin() == false) {
    //     delay(500);
    //     Serial.printf("CS Pin State: %d\n", digitalRead(config.mag_config.cs_pin));
    //     if (!ICM_MAG.softReset()) {
    //         Serial.println("magnetometer did not reset...");
    //     } else {
    //         Serial.println("successful reset...");
    //     }
    //     delay(500);
    // }
    while (ICM_MAG.begin(config.mag_config.cs_pin, mag_spi_settings) == false) {
        delay(500);
        if (!ICM_MAG.softReset()) {
            Serial.println("magnetometer did not reset...");
        } else {
            Serial.println("successful reset...");
        }
        if (ICM_MAG.is3WireSPIEnabled()) {
            Serial.println("3Wire SPI enabled.");
        }
        if (!ICM_MAG.mmc_io.spiInUse()) {
            Serial.println("SPI not being set properly");
        }
        delay(500);
    }
    // ICM_MAG.begin(config.mag_config.cs_pin, mag_spi_settings);

    // Setup magnetometer
    ICM_MAG.softReset();
    ICM_MAG.setFilterBandwidth(800);
    ICM_MAG.setContinuousModeFrequency(1000);
    ICM_MAG.enableAutomaticSetReset();
    ICM_MAG.enableContinuousMode();
    ICM_MAG.enableInterrupt();
    newMagData = true;

    // Set interrupt flags
    imu_use_interrupts = true;
    mag_use_interrupts = true;

    // Setu
    return true;
}

void ICM42670_MMC5983_IMU::getData() {
    //TODO: write method to get data from the imu + magnetometer
    // Get data from the IMU and save it
    // accl: x,y,z
    // gyro: x,y,z
    // mag: x,y,z

    if (!imu_use_interrupts) {
        // Read raw imu data
        inv_imu_sensor_event_t imu_event;
        ICM_IMU.getDataFromRegisters(imu_event);    

        // Save data to class
        accl[0] = -((float)imu_event.accel[0] * accelSensitivity);
        accl[1] = -((float)imu_event.accel[1] * accelSensitivity);
        accl[2] = ((float)imu_event.accel[2] * accelSensitivity);
        
        gyro[0] = (float)imu_event.gyro[0] * gyroMultipier;
        gyro[1] = (float)imu_event.gyro[1] * gyroMultipier;
        gyro[2] = (float)imu_event.gyro[2] * gyroMultipier;
    } else if (newIMUData) {
        newIMUData = false;
        ICM_IMU.getDataFromFifo(imu_cb);
        // Save accel data
        accl[0] = ((float)imu_accl[0] * accelSensitivity);
        accl[1] = ((float)imu_accl[1] * accelSensitivity);
        accl[2] = ((float)imu_accl[2] * accelSensitivity);
        Serial.printf("New accel data: %f, %f, %f\n", accl[0], accl[1], accl[2]); // for debugging

        // Save gyro data
        gyro[0] = (float)imu_gyro[0] * gyroMultipier;
        gyro[1] = (float)imu_gyro[1] * gyroMultipier;
        gyro[2] = (float)imu_gyro[2] * gyroMultipier;
    }

    // Magnetometer
    uint32_t rawMagX, rawMagY, rawMagZ;
    if (newMagData || !mag_use_interrupts) {
        newMagData = false;
        ICM_MAG.clearMeasDoneInterrupt();
        ICM_MAG.readFieldsXYZ(&rawMagX, &rawMagY, &rawMagZ);
        // Save magnetometer data to class
        
        magn[0] = -((float)rawMagX - 131072.0);
        magn[1] = -((float)rawMagY - 131072.0);
        magn[2] = (float)rawMagZ - 131072.0;
        Serial.printf("New Magnetometer Data: %f, %f, %f\n",magn[0], magn[1], magn[2]); // Debugging
    }
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