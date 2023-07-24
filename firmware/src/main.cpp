//****************************************************************************//
// T-Stick - sopranino/soprano firmware                                       //
// SAT/Metalab                                                                //
// Input Devices and Music Interaction Laboratory (IDMIL), McGill University  //
// Edu Meneses (2022) - https://www.edumeneses.com                            //
//****************************************************************************//

/* Created using the Puara template: https://github.com/Puara/puara-module-template 
 * The template contains a fully commented version for the commonly used commands 
 */


unsigned int firmware_version = 230801;

// set the amount of capacitive stripes for the sopranino (15) or soprano (30)
#define TSTICK_SIZE 60

/*
  Choose the capacitive sensing board
  - Trill
  - IDMIL Capsense board
*/
#define touch_TRILL
// #define touch_CAPSENSE

/*
 Define libmapper and OSC
*/
#define LIBMAPPER
#define OSC

/*
Define microcontroller
//#define SPARKFUN_ESP32_THING_PLUS 1

*/

/*
Select SDA and SCL Pins based on Micro controllers
*/
#ifdef SPARKFUN_ESP32_THING_PLUS
const int SDA_PIN = 23;
const int SCL_PIN = 22;
#else
const int SDA_PIN = 21;
const int SCL_PIN = 22;
#endif


#include "Arduino.h"

// For disabling power saving
#include "esp_wifi.h"

#include <puara.h>
#include <puara_gestures.h>
#include <mapper.h>

#include <deque>
#include <cmath>
#include <algorithm>
#include <numeric>

// Include task scheduler library
#include <TaskScheduler.h>

// initializing libmapper, puara, puara-gestures, and liblo client
mpr_dev lm_dev = 0;
Puara puara;
PuaraGestures gestures;
lo_address osc1;
lo_address osc2;
std::string baseNamespace = "/";
std::string oscNamespace;

/////////////////////
// Pin definitions //
/////////////////////

struct Pin {
    int led;     // Built In LED pin
    int battery; // To check battery level (voltage)
    int fsr;
    int button;
};

#ifdef ARDUINO_LOLIN_D32_PRO
    Pin pin{ 5, 35, 33, 15 };
#elif defined(ARDUINO_TINYPICO)
    #include "TinyPICO.h"
    Pin pin{ 5, 35, 33, 15 };
    TinyPICO tinypico = TinyPICO();
#endif

//////////////////////////////////
// Battery struct and functions //
//////////////////////////////////
  
struct BatteryData {
    unsigned int percentage = 0;
    unsigned int lastPercentage = 0;
    float value;
    unsigned long timer = 0;
    int interval = 1000; // in ms (1/f)
    int queueAmount = 10; // # of values stored
    std::deque<int> filterArray; // store last values
} battery;

// read battery level (based on https://www.youtube.com/watch?v=yZjpYmWVLh8&feature=youtu.be&t=88) 
void readBattery() {
    #ifdef ARDUINO_LOLIN_D32_PRO
        battery.value = analogRead(pin.battery) / 4096.0 * 7.445;
    #elif defined(ARDUINO_TINYPICO)
        battery.value = tinypico.GetBatteryVoltage();
    #endif
    battery.percentage = static_cast<int>((battery.value - 2.9) * 100 / (4.15 - 2.9));
    if (battery.percentage > 100)
        battery.percentage = 100;
    if (battery.percentage < 0)
        battery.percentage = 0;
}

void batteryFilter() {
    battery.filterArray.push_back(battery.percentage);
    if(battery.filterArray.size() > battery.queueAmount) {
        battery.filterArray.pop_front();
    }
    battery.percentage = 0;
    for (int i=0; i<battery.filterArray.size(); i++) {
        battery.percentage += battery.filterArray.at(i);
    }
    battery.percentage /= battery.filterArray.size();
}
///////////////////////////////////
// Include sensor manager files  //
///////////////////////////////////

#include "sensormanager.h"

sensorManager sensormanager;

///////////////////////////////////
// Include button function files //
///////////////////////////////////

#include "button.h"

Button button;

////////////////////////////////
// Include FSR function files //
////////////////////////////////

#include "fsr.h"

Fsr fsr;

////////////////////////////////
// Include IMU function files //
////////////////////////////////

#include <imu.h>
IMU imu;

////////////////////////////////
// Include Fuel Gauge function files //
////////////////////////////////

#include <fuelgauge.h>
// //#include <fuelgauge2.h> if using LC709203F
FuelGauge fuelgauge;

//////////////////////////////////////////////
// Include Touch stuff                      //
//////////////////////////////////////////////

#ifdef touch_TRILL
  #include "touch.h"
  Touch touch;
  Touch touch2;
#endif

#ifdef touch_CAPSENSE
  #include "capsense.h"
  Capsense capsense;
#endif

////////////////////////////////
// Include LED function files //
////////////////////////////////

#include "led.h"

Led led;

struct Led_variables {
    int ledValue = 0;
    uint8_t color = 0;
} led_var;

//////////////////////
// Liblo OSC server //
//////////////////////

void error(int num, const char *msg, const char *path) {
    printf("Liblo server error %d in path %s: %s\n", num, path, msg);
    fflush(stdout);
}
lo_server_thread osc_server;

int generic_handler(const char *path, const char *types, lo_arg ** argv,
                    int argc, lo_message data, void *user_data) {
    for (int i = 0; i < argc; i++) {
        printf("arg %d '%c' ", i, types[i]);
        lo_arg_pp((lo_type)types[i], argv[i]);
        printf("\n");
    }
    printf("\n");
    fflush(stdout);

    return 1;
}

////////////////////////////////
// sensors and libmapper data //
////////////////////////////////

struct Lm {
    mpr_sig fsr = 0;
    float fsrMax = 4900;
    float fsrMin = 0;
    mpr_sig accel = 0;
    float accelMax[3] = {50, 50, 50};
    float accelMin[3] = {-50, -50, -50};
    mpr_sig gyro = 0;
    float gyroMax[3] = {25, 25, 25};
    float gyroMin[3] = {-25, -25, -25};
    mpr_sig magn = 0;
    float magnMax[3] = {25, 25, 25};
    float magnMin[3] = {-25, -25, -25};
    mpr_sig quat = 0;
    float quatMax[4] = {1, 1, 1, 1};
    float quatMin[4] = {-1, -1, -1, -1};
    mpr_sig ypr = 0;
    float yprMax[3] = {180, 180, 180};
    float yprMin[3] = {-180, -180, -180};
    mpr_sig shake = 0;
    float shakeMax[3] = {50, 50, 50};
    float shakeMin[3] = {-50, -50, -50};
    mpr_sig jab = 0;
    float jabMax[3] = {50, 50, 50};
    float jabMin[3] = {-50, -50, -50};
    mpr_sig brush = 0;
    mpr_sig multibrush = 0;
    float brushMax[4] = {50, 50, 50, 50};
    float brushMin[4] = {-50, -50, -50, -50};
    mpr_sig rub = 0;
    mpr_sig multirub = 0;
    float rubMax[4] = {50, 50, 50, 50};
    float rubMin[4] = {-50, -50, -50, -50};
    mpr_sig touch = 0;
    int touchMax[TSTICK_SIZE]; // Initialized in setup()
    int touchMin[TSTICK_SIZE];
    mpr_sig count = 0;
    int countMax = 100;
    int countMin = 0;
    mpr_sig tap = 0;
    mpr_sig ttap = 0;
    mpr_sig dtap = 0;
    int tapMax = 1;
    int tapMin = 0;
    mpr_sig bat = 0;
    int batMax = 100;
    int batMin = 0;
} lm;

struct Sensors {
    float accl [3];
    float gyro [3];
    float magn [3];
    float quat [4];
    float ypr [3];
    float shake [3];
    float jab [3];
    float brush;
    float rub;
    float multibrush [4];
    float multirub [4];
    int count;
    int tap;
    int dtap;
    int ttap;
    int fsr;
    float battery;
    float chargerate;
} sensors;

struct Event {
    bool shake = false;
    bool jab = false;
    bool count = false;
    bool tap = false;
    bool dtap = false;
    bool ttap = false;
    bool fsr = false;
    bool brush = false;
    bool rub = false;
    bool battery;
} event;

//*********************TASK SCHEDULING*******************************************//
// Timing variables in t = 1/f
const uint32_t LIBMAPPER_UPDATE_RATE =10 * TASK_MILLISECOND; // f = 100Hz
const uint32_t OSC_UPDATE_RATE = 10 * TASK_MILLISECOND; // f = 100Hz
const uint32_t SENSOR_READ_RATE = TASK_MILLISECOND; // f = 1000Hz
const uint32_t BATTERY_READ_RATE = 30 * TASK_SECOND; // t = 1s
const uint32_t POWER_STATUS_RATE = 10 * TASK_MILLISECOND; // f = 100Hz
const uint32_t MAINTENANCE_RATE = 30 * TASK_SECOND; // t = 30s
const uint32_t I2CUPDATE_FREQ = 3400000; // high speed mode;

// Communication Functions
#ifdef LIBMAPPER
void updateLibmapper();
void mapperFSR();
void mapperIMU();
void mapperTouch();
void mapperInertialGestures();
void mapperTouchGestures();
void mapperButtonGestures();
void mapperBattery();
#endif
#ifdef OSC
void updateOSC();
void oscFSR();
void oscIMU();
void oscTouch();
void oscInertialGestures();
void oscTouchGestures();
void oscButtonGestures();
void oscBattery();
#endif
// Sensor functions
// Fast sensor functions
void readSensors();
void getTouchData();
void getIMUData();
void updateSensors();

// Slow sensor functions
void getFuelGaugeData();
void updatePowerStatus();
void updateLED();

// Maintenance Functions
void scanInactiveSensors();
void scanActiveSensors();

// Create Scheduler
Scheduler runnerTstick;

// Timing methods for testing
// Callback methods prototypes
bool tOn(); void tOff();
std::map<unsigned int, std::vector<int>> taskStarts;
std::map<unsigned int, std::vector<int>> taskEnds;
unsigned long c1, c2;
std::vector<int> tmp;

// //==========Tasks to check overhead===========//
// // Fast Sensor Tasks
// Task UpdateSensors (SENSOR_READ_RATE, 1000, &readSensors, &runnerTstick, false, &tOn, &tOff);

// // Slow Sensor Tasks
// Task BatteryUpdate (BATTERY_READ_RATE, 1000, &getFuelGaugeData, &runnerTstick, false, &tOn, &tOff);
// Task PowerStatusUpdate (POWER_STATUS_RATE, 1000, &updatePowerStatus, &runnerTstick, false, &tOn, &tOff);

// // Comms Tasks
// #ifdef LIBMAPPER
// Task libmapperUpdate (LIBMAPPER_UPDATE_RATE, 1000, &updateLibmapper, &runnerTstick,false, &tOn, &tOff);
// #endif
// #ifdef OSC
// Task OSCUpdate (OSC_UPDATE_RATE, 1000, &updateOSC, &runnerTstick,false, &tOn, &tOff);
// #endif
// // Maintenance Tasks
// Task SensorsScan (MAINTENANCE_RATE, 10, &scanInactiveSensors, &runnerTstick, false, &tOn, &tOff);

//==========T-Stick Tasks===========//
// Fast Sensor Tasks
Task UpdateSensors (SENSOR_READ_RATE, 1000, &getTouchData, &runnerTstick, true);

// Slow Sensor Tasks
Task BatteryUpdate (BATTERY_READ_RATE, TASK_FOREVER, &getFuelGaugeData, &runnerTstick, true);
Task PowerStatusUpdate (POWER_STATUS_RATE, TASK_FOREVER, &updatePowerStatus, &runnerTstick, true);

// Comms Tasks
#ifdef LIBMAPPER
Task libmapperUpdate (LIBMAPPER_UPDATE_RATE, TASK_FOREVER, &updateLibmapper, &runnerTstick,true);
#endif
#ifdef OSC
Task OSCUpdate (OSC_UPDATE_RATE, TASK_FOREVER, &oscFSR, &runnerTstick,true);
#endif
// Maintenance Tasks
Task SensorsScan (MAINTENANCE_RATE, 10, &scanInactiveSensors, &runnerTstick, true);
//==========Functions for task scheduler===========//
void saveStart() {
    // Get current task
    Scheduler &s = Scheduler::currentScheduler();
    Task &t = s.currentTask();

    c1 = millis();
    taskStarts[t.getId()].push_back(c1);
}

void saveEnd() {
    // Get current task
    Scheduler &s = Scheduler::currentScheduler();
    Task &t = s.currentTask();

    c2 = millis();
    taskEnds[t.getId()].push_back(c2);
}
bool tOn() {
    // Get current task
    Scheduler &s = Scheduler::currentScheduler();
    Task &t = s.currentTask();

    // Get timing
    taskStarts[t.getId()] = tmp;
    taskEnds[t.getId()] = tmp;
    return true;
}

void tOff() {
    // Get current task
    Scheduler &s = Scheduler::currentScheduler();
    Task &t = s.currentTask();

    // Create arrays
    std::vector<int> durationArray;
    std::vector<int> periodArray;
    std::vector<int> tmpPeriod;
    tmpPeriod = taskStarts[t.getId()];
    tmpPeriod[0] = 0;

    // Calculate average duration and frequency
    std::transform(taskEnds[t.getId()].begin(), taskEnds[t.getId()].end(), taskStarts[t.getId()].begin(), std::back_inserter(durationArray), std::minus<int>());
    std::transform(taskStarts[t.getId()].begin(), taskStarts[t.getId()].end(), tmpPeriod.begin(), std::back_inserter(periodArray), std::minus<int>());
    
    // Get average
    periodArray.erase(periodArray.begin());
    float dur = std::accumulate(durationArray.begin(),durationArray.end(), 0.0)/ durationArray.size();
    float per = std::accumulate(periodArray.begin(), periodArray.end(), 0.0)/ periodArray.size();
    
    std::cout
    << "Task " << t.getId() << "done.\n" 
    << "Average Period = " << per << "\n"
    << "Duration = " << dur << "\n"
    << std::endl;
}

//================Maintenance Functions============//
void scanInactiveSensors() {
    // saveStart();
    // Scan inactive sensors for the sensor manager
    if (sensormanager.getnumInactiveSensors() != 0) {
        sensormanager.scanInactiveI2C();
    } else if (sensormanager.getnumActiveSensors() == 0) { 
        // No active and inactive sensors means that asll sensors are disabled
        std::cout << "All sensors are disabled. Skipping inactive sensor scan" << std::endl;
    } else {
        // No inactive sensors, but some active or disabled
        std::cout << "All sensors are either active or disabled. Skipping inactive sensor scan" << std::endl;
    }
    
    // Set active sensor scan and schedule it immediately
    SensorsScan.setCallback(&scanActiveSensors);
    SensorsScan.forceNextIteration();
}

void scanActiveSensors() {
    // Scan active sensors for the sensor manager
    if (sensormanager.getnumActiveSensors() != 0) {
        sensormanager.scanActiveI2C();
    } else if (sensormanager.getnumInactiveSensors() == 0) { 
        // No active and inactive sensors means that asll sensors are disabled
        std::cout << "All sensors are disabled. Skipping active sensor scan" << std::endl;
    } else {
        // No inactive sensors, but some active or disabled
        std::cout << "All sensors are either inactive or disabled. Skipping active sensor scan" << std::endl;
    }
    // saveEnd();

    // Set inactive sensor as next call
    SensorsScan.setCallback(&scanInactiveSensors);
}

void updatePowerStatus() {
    // saveStart();
    // go to deep sleep if double press button
    if (gestures.getButtonDTap()){
        std::cout << "\nEntering deep sleep.\n\nGoodbye!\n" << std::endl;
        delay(1000);
        esp_deep_sleep_start();
    }
    // saveEnd();
}
//================Sensor Functions=================//
void readSensors() {
    // Save start time
    // saveStart();

    // Get button and fsr data
    button.readButton();
    fsr.readFsr();

    // Read touch data next
    UpdateSensors.setCallback(&getTouchData);
    UpdateSensors.forceNextIteration();
}

void getTouchData() {
    // Get the Sensor data
    if (sensormanager.checkSensorStatus("trillcraft1") && (sensormanager.status == 1)) {
        sensormanager.getSensorData("trillcraft1");
    } else {
        //TouchUpdate.disable();
    }

    // Read IMU Data
    UpdateSensors.setCallback(&getIMUData);
    UpdateSensors.forceNextIteration();
}

void getIMUData() {
    // Get sensor data if the sensor is active
    if (sensormanager.checkSensorStatus("imu") && (sensormanager.status == 1)) {
        // Get the Sensor data
        sensormanager.getSensorData("imu");

        // Save IMU data to gestures array do this here as update inertial gestures crashes if buffers are empty
        gestures.setAccelerometerValues(imu.ax,
                                        imu.ay,
                                        imu.az);
        gestures.setGyroscopeValues(imu.gx,
                                    imu.gy,
                                    imu.gz);
        gestures.setMagnetometerValues(imu.mx,
                                    imu.my,
                                    imu.mz);    
        gestures.updateInertialGestures();                                   
    } else {
        //IMUUpdate.disable();
    }
    
    // Update Sensor array
    UpdateSensors.setCallback(&updateSensors);
    UpdateSensors.forceNextIteration();
}

void updateSensors() { 
    if (sensormanager.status == 1) {
        // //Gesture update
        gestures.updateTrigButton(button.getButton());

        // Update Sensor Structure for outputting
        // Preparing arrays for libmapper signals
        sensors.fsr = fsr.getCookedValue();
        // Convert accel from g's to meters/sec^2
        sensors.accl[0] = gestures.getAccelX() * 9.80665;
        sensors.accl[1] = gestures.getAccelY() * 9.80665;
        sensors.accl[2] = gestures.getAccelZ() * 9.80665;
        // Convert gyro from degrees/sec to radians/sec
        sensors.gyro[0] = gestures.getGyroX() * M_PI / 180;
        sensors.gyro[1] = gestures.getGyroY() * M_PI / 180;
        sensors.gyro[2] = gestures.getGyroZ() * M_PI / 180;
        // Convert mag from Gauss to uTesla
        sensors.magn[0] = gestures.getMagX() / 10000;
        sensors.magn[1] = gestures.getMagY() / 10000;
        sensors.magn[2] = gestures.getMagZ() / 10000;
        // Orientation quaternion
        sensors.quat[0] = gestures.getOrientationQuaternion().w;
        sensors.quat[1] = gestures.getOrientationQuaternion().x;
        sensors.quat[2] = gestures.getOrientationQuaternion().y;
        sensors.quat[3] = gestures.getOrientationQuaternion().z;
        // Yaw (heading), pitch (tilt) and roll
        sensors.ypr[0] = gestures.getYaw();
        sensors.ypr[1] = gestures.getPitch();
        sensors.ypr[2] = gestures.getRoll();
        if (sensors.shake[0] != gestures.getShakeX() || sensors.shake[1] != gestures.getShakeY() || sensors.shake[2] != gestures.getShakeZ()) {
            sensors.shake[0] = gestures.getShakeX();
            sensors.shake[1] = gestures.getShakeY();
            sensors.shake[2] = gestures.getShakeZ();
            event.shake = true;
        } else { event.shake = false; }
        if (sensors.brush != gestures.brush || sensors.multibrush[0] != gestures.multiBrush[0]) {
            sensors.brush = gestures.brush;
            sensors.multibrush[0] = gestures.multiBrush[0];
            sensors.multibrush[1] = gestures.multiBrush[1];
            sensors.multibrush[2] = gestures.multiBrush[2];
            sensors.multibrush[3] = gestures.multiBrush[3];
            event.brush = true;
        } else { event.brush = false; }
        if (sensors.rub != gestures.rub || sensors.multirub[0] != gestures.multiRub[0]) {
            sensors.rub = gestures.rub;
            sensors.multirub[0] = gestures.multiRub[0];
            sensors.multirub[1] = gestures.multiRub[1];
            sensors.multirub[2] = gestures.multiRub[2];
            sensors.multirub[3] = gestures.multiRub[3];
            event.rub = true;
        } else { event.rub = false; }
        if (sensors.jab[0] != gestures.getJabX() || sensors.jab[1] != gestures.getJabY() || sensors.jab[2] != gestures.getJabZ()) {
            sensors.jab[0] = gestures.getJabX();
            sensors.jab[1] = gestures.getJabY();
            sensors.jab[2] = gestures.getJabZ();
            event.jab = true;
        } else { event.jab = false; }
        if (sensors.count != gestures.getButtonCount()) {sensors.count = gestures.getButtonCount(); event.count = true; } else { event.count = false; }
        if (sensors.tap != gestures.getButtonTap()) {sensors.tap = gestures.getButtonTap(); event.tap = true; } else { event.tap = false; }
        if (sensors.dtap != gestures.getButtonDTap()) {sensors.dtap = gestures.getButtonDTap(); event.dtap = true; } else { event.dtap = false; }
        if (sensors.ttap != gestures.getButtonTTap()) {sensors.ttap = gestures.getButtonTTap(); event.ttap = true; } else { event.ttap = false; }

        // Get battery reading from fuel gauge if enabled
        if (sensormanager.checkSensorStatus("battery")) {
            // Update battery information (including charge rate)
            if (sensors.battery != fuelgauge.percentage) {sensors.battery = fuelgauge.percentage; event.battery = true; } else { event.battery = false; };
            if (sensors.chargerate != fuelgauge.chargerate) {sensors.battery = fuelgauge.chargerate; event.battery = true; } else { event.battery = false; };
        } else {
            if (sensors.battery != battery.percentage) {sensors.battery = battery.percentage; event.battery = true; } else { event.battery = false; };
        }
    } else {
        std::cout << "Sensor manager was not initialised.\nSkipping reading sensor data. Check sensor configs were correct.\n" << std::endl;
    }
    // Stop reading sensor data
    // saveEnd();

    // End sensor read task and reset to readSensors
    UpdateSensors.setCallback(&readSensors);
}
void getFuelGaugeData() {
    saveStart();
    // Get the Sensor data
    if (sensormanager.checkSensorStatus("battery") && (sensormanager.status == 1)) {
        sensormanager.getSensorData("battery");
    } else {
        // // disable task if sensor is inactive
        // BatteryUpdate.disable();
        readBattery();
        batteryFilter();
    }
    // Update the LED afterwards
    BatteryUpdate.setCallback(&updateLED);
    BatteryUpdate.forceNextIteration();
}

void updateLED() {
    // Set LED - connection status and battery level
    #ifdef ARDUINO_LOLIN_D32_PRO
        if (battery.percentage < 10) {        // low battery - flickering
        led.setInterval(75);
        led_var.ledValue = led.blink(255, 50);
        ledcWrite(0, led_var.ledValue);
        } else {
            if (puara.get_StaIsConnected()) { // blinks when connected, cycle when disconnected
                led.setInterval(1000);
                led_var.ledValue = led.blink(255, 40);
                ledcWrite(0, led_var.ledValue);
            } else {
                led.setInterval(4000);
                led_var.ledValue = led.cycle(led_var.ledValue, 0, 255);
                ledcWrite(0, led_var.ledValue);
            }
        }
    #elif defined(ARDUINO_TINYPICO)
        if (battery.percentage < 10) {                // low battery (red)
            led.setInterval(20);
            led_var.color = led.blink(255, 20);
            tinypico.DotStar_SetPixelColor(led_var.color, 0, 0);
        } else {
            if (puara.get_StaIsConnected()) {         // blinks when connected, cycle when disconnected
                led.setInterval(1000);                // RGB: 0, 128, 255 (Dodger Blue)
                led_var.color = led.blink(255,20);
                tinypico.DotStar_SetPixelColor(0, uint8_t(led_var.color/2), led_var.color);
            } else {
                led.setInterval(4000);
                led_var.color = led.cycle(led_var.color, 0, 255);
                tinypico.DotStar_SetPixelColor(0, uint8_t(led_var.color/2), led_var.color);
            }
        }
    #endif  

    // Set new callback and force the next iteration
    BatteryUpdate.setCallback(&mapperBattery);
    BatteryUpdate.forceNextIteration();
}
//=================COMMS Functions=================//
#ifdef LIBMAPPER
void updateLibmapper () {    
    // saveStart();
    // Poll libmapper
    mpr_dev_poll(lm_dev, 0);

    // Set next libmapper task
    libmapperUpdate.setCallback(&mapperFSR);
    libmapperUpdate.forceNextIteration();
}

void mapperFSR() {        
    // Update libmapper outputs
    // updating libmapper signals
    mpr_sig_set_value(lm.fsr, 0, 1, MPR_FLT, &sensors.fsr);

    // Set new Callback and force next iteration
    libmapperUpdate.setCallback(&mapperIMU);
    libmapperUpdate.forceNextIteration();
}

void mapperIMU() {    
    // Update libmapper outputs
    // updating libmapper signals
    mpr_sig_set_value(lm.accel, 0, 3, MPR_FLT, &sensors.accl);
    mpr_sig_set_value(lm.gyro, 0, 3, MPR_FLT, &sensors.gyro);
    mpr_sig_set_value(lm.magn, 0, 3, MPR_FLT, &sensors.magn);
    mpr_sig_set_value(lm.quat, 0, 4, MPR_FLT, &sensors.quat);
    mpr_sig_set_value(lm.ypr, 0, 3, MPR_FLT, &sensors.ypr);

    // Set new Callback and force next iteration
    libmapperUpdate.setCallback(&mapperTouch);
    libmapperUpdate.forceNextIteration();
}

void mapperTouch() {    
    // Update libmapper outputs
    // updating libmapper signals
    // Libmapper signals
    #ifdef touch_TRILL
        mpr_sig_set_value(lm.touch, 0, touch.touchSize, MPR_INT32, &touch.touch);
    #endif
    #ifdef touch_CAPSENSE
        mpr_sig_set_value(lm.touch, 0, capsense.touchStripsSize, MPR_INT32, &capsense.data);
    #endif
    // saveEnd();

    // Set new Callback and force next iteration
    libmapperUpdate.setCallback(&mapperInertialGestures);
    libmapperUpdate.forceNextIteration();
}

void mapperInertialGestures() {    
    // Update libmapper outputs
    // updating libmapper signals
    mpr_sig_set_value(lm.shake, 0, 3, MPR_FLT, &sensors.shake);
    mpr_sig_set_value(lm.jab, 0, 3, MPR_FLT, &sensors.jab);

    // Set new Callback and force next iteration
    libmapperUpdate.setCallback(&mapperTouchGestures);
    libmapperUpdate.forceNextIteration();
}

void mapperTouchGestures() {    
    // Update libmapper outputs
    // updating libmapper signals
    mpr_sig_set_value(lm.rub, 0, 1, MPR_FLT, &sensors.rub);
    mpr_sig_set_value(lm.brush, 0, 1, MPR_FLT, &sensors.brush);
    mpr_sig_set_value(lm.multirub, 0, 4, MPR_FLT, &sensors.multirub);
    mpr_sig_set_value(lm.multibrush, 0, 4, MPR_FLT, &sensors.multibrush);

    // Libmapper signals
    #ifdef touch_TRILL
        mpr_sig_set_value(lm.touch, 0, touch.touchSize, MPR_INT32, &touch.touch);
    #endif
    #ifdef touch_CAPSENSE
        mpr_sig_set_value(lm.touch, 0, capsense.touchStripsSize, MPR_INT32, &capsense.data);
    #endif
    // saveEnd();

    // Set new Callback and force next iteration
    libmapperUpdate.setCallback(&mapperButtonGestures);
    libmapperUpdate.forceNextIteration();
}

void mapperButtonGestures() {    
    // Update libmapper outputs
    // updating libmapper signals
    mpr_sig_set_value(lm.count, 0, 1, MPR_INT32, &sensors.count);
    mpr_sig_set_value(lm.tap, 0, 1, MPR_INT32, &sensors.tap);
    mpr_sig_set_value(lm.ttap, 0, 1, MPR_INT32, &sensors.dtap);
    mpr_sig_set_value(lm.dtap, 0, 1, MPR_INT32, &sensors.ttap);

    // Set Callback back to beginning
    libmapperUpdate.setCallback(&updateLibmapper);
}

void mapperBattery() {    
    // Update libmapper outputs
    // updating libmapper signals
    mpr_sig_set_value(lm.bat, 0, 1, MPR_FLT, &sensors.battery);

    // Update Callback
    BatteryUpdate.setCallback(&oscBattery);
    BatteryUpdate.forceNextIteration();
}
#endif

#ifdef OSC
void oscFSR() {
    // saveStart();
    // Sending continuous OSC messages
    if (puara.IP1_ready()) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "raw/fsr");
            lo_send(osc1, oscNamespace.c_str(), "i", sensors.fsr);
    }
    if (puara.IP2_ready()) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "raw/fsr");
            lo_send(osc2, oscNamespace.c_str(), "i", sensors.fsr);
    }

    // Set new callback and force next iteration
    OSCUpdate.setCallback(&oscIMU);
    OSCUpdate.forceNextIteration();
}

void oscIMU() {
    // saveStart();
    // Sending continuous OSC messages
    if (puara.IP1_ready()) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "raw/accl");
            lo_send(osc1, oscNamespace.c_str(), "fff", sensors.accl[0], sensors.accl[1], sensors.accl[2]);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "raw/gyro");
            lo_send(osc1, oscNamespace.c_str(), "fff", sensors.gyro[0], sensors.gyro[1], sensors.gyro[2]);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "raw/magn");
            lo_send(osc1, oscNamespace.c_str(), "fff", sensors.magn[0], sensors.magn[1], sensors.magn[2]);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "orientation");
            lo_send(osc1, oscNamespace.c_str(), "ffff", sensors.quat[0], sensors.quat[1], sensors.quat[2], sensors.quat[3]);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "ypr");
            lo_send(osc1, oscNamespace.c_str(), "fff", sensors.ypr[0], sensors.ypr[1], sensors.ypr[2]);
    }
    if (puara.IP2_ready()) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "raw/accl");
            lo_send(osc2, oscNamespace.c_str(), "fff", sensors.accl[0], sensors.accl[1], sensors.accl[2]);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "raw/gyro");
            lo_send(osc2, oscNamespace.c_str(), "fff", sensors.gyro[0], sensors.gyro[1], sensors.gyro[2]);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "raw/magn");
            lo_send(osc2, oscNamespace.c_str(), "fff", sensors.magn[0], sensors.magn[1], sensors.magn[2]);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "orientation");
            lo_send(osc2, oscNamespace.c_str(), "ffff", sensors.quat[0], sensors.quat[1], sensors.quat[2], sensors.quat[3]);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "ypr");
            lo_send(osc2, oscNamespace.c_str(), "fff", sensors.ypr[0], sensors.ypr[1], sensors.ypr[2]);
    }
    // Set new callback and force next iteration
    OSCUpdate.setCallback(&oscTouch);
    OSCUpdate.forceNextIteration();
}

void oscTouch() {
    // saveStart();
    // Sending continuous OSC messages
    if (puara.IP1_ready()) {

            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "raw/capsense");
            #ifdef touch_TRILL
                if (TSTICK_SIZE == 30) {
                    lo_send(osc1, oscNamespace.c_str(), "iiiiiiiiiiiiiiiiiiiiiiiiiiiiii", touch.touch[0], touch.touch[1],touch.touch[2],
                    touch.touch[3],touch.touch[4],touch.touch[5], touch.touch[6], touch.touch[7], touch.touch[8],
                    touch.touch[9], touch.touch[10], touch.touch[11], touch.touch[12], touch.touch[13], touch.touch[14], touch.touch[15], touch.touch[16],touch.touch[17],
                    touch.touch[18],touch.touch[19],touch.touch[20], touch.touch[21], touch.touch[22], touch.touch[23],
                    touch.touch[24], touch.touch[25], touch.touch[26], touch.touch[27], touch.touch[28], touch.touch[29]);
                } else if (TSTICK_SIZE == 45) {
                    // Send data from the first board
                    lo_send(osc1, oscNamespace.c_str(), "iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii", touch.touch[0], touch.touch[1],touch.touch[2],
                    touch.touch[3],touch.touch[4],touch.touch[5], touch.touch[6], touch.touch[7], touch.touch[8],
                    touch.touch[9], touch.touch[10], touch.touch[11], touch.touch[12], touch.touch[13], touch.touch[14], touch.touch[15], touch.touch[16],touch.touch[17],
                    touch.touch[18],touch.touch[19],touch.touch[20], touch.touch[21], touch.touch[22], touch.touch[23],
                    touch.touch[24], touch.touch[25], touch.touch[26], touch.touch[27], touch.touch[28], touch.touch[29], touch2.touch[0], touch2.touch[1],touch2.touch[2],
                    touch2.touch[3],touch2.touch[4],touch2.touch[5], touch2.touch[6], touch2.touch[7], touch2.touch[8],
                    touch2.touch[9], touch2.touch[10], touch2.touch[11], touch2.touch[12], touch2.touch[13], touch2.touch[14]);
                } else if (TSTICK_SIZE == 60) {
                    // Send data from the first board
                    lo_send(osc1, oscNamespace.c_str(), "iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii", touch.touch[0], touch.touch[1],touch.touch[2],
                    touch.touch[3],touch.touch[4],touch.touch[5], touch.touch[6], touch.touch[7], touch.touch[8],
                    touch.touch[9], touch.touch[10], touch.touch[11], touch.touch[12], touch.touch[13], touch.touch[14], touch.touch[15], touch.touch[16],touch.touch[17],
                    touch.touch[18],touch.touch[19],touch.touch[20], touch.touch[21], touch.touch[22], touch.touch[23],
                    touch.touch[24], touch.touch[25], touch.touch[26], touch.touch[27], touch.touch[28], touch.touch[29], touch2.touch[0], touch2.touch[1],touch2.touch[2],
                    touch2.touch[3],touch2.touch[4],touch2.touch[5], touch2.touch[6], touch2.touch[7], touch2.touch[8],
                    touch2.touch[9], touch2.touch[10], touch2.touch[11], touch2.touch[12], touch2.touch[13], touch2.touch[14], touch2.touch[15], touch2.touch[16],touch2.touch[17],
                    touch2.touch[18],touch2.touch[19],touch2.touch[20], touch2.touch[21], touch2.touch[22], touch2.touch[23],
                    touch2.touch[24], touch2.touch[25], touch2.touch[26], touch2.touch[27], touch2.touch[28], touch2.touch[29]);
                }
                else {
                    lo_send(osc1, oscNamespace.c_str(), "iiiiiiiiiiiiiii", touch.touch[0], touch.touch[1],touch.touch[2],
                        touch.touch[3],touch.touch[4],touch.touch[5], touch.touch[6], touch.touch[7], touch.touch[8],
                        touch.touch[9], touch.touch[10], touch.touch[11], touch.touch[12], touch.touch[13], touch.touch[14]);
                }
            #endif
            #ifdef touch_CAPSENSE
                lo_send(osc1, oscNamespace.c_str(), "iiiiiiiiiiiiiiii", capsense.data[0], capsense.data[1],capsense.data[2],
                    capsense.data[3],capsense.data[4],capsense.data[5], capsense.data[6], capsense.data[7], capsense.data[8],
                    capsense.data[9], capsense.data[10], capsense.data[11], capsense.data[12], capsense.data[13], capsense.data[14],
                    capsense.data[15]
            );
            #endif
    }
    if (puara.IP2_ready()) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "raw/capsense");
            #ifdef touch_TRILL
                if (TSTICK_SIZE == 30) {
                    lo_send(osc2, oscNamespace.c_str(), "iiiiiiiiiiiiiiiiiiiiiiiiiiiiii", touch.touch[0], touch.touch[1],touch.touch[2],
                    touch.touch[3],touch.touch[4],touch.touch[5], touch.touch[6], touch.touch[7], touch.touch[8],
                    touch.touch[9], touch.touch[10], touch.touch[11], touch.touch[12], touch.touch[13], touch.touch[14], touch.touch[15], touch.touch[16],touch.touch[17],
                    touch.touch[18],touch.touch[19],touch.touch[20], touch.touch[21], touch.touch[22], touch.touch[23],
                    touch.touch[24], touch.touch[25], touch.touch[26], touch.touch[27], touch.touch[28], touch.touch[29]);
                } else if (TSTICK_SIZE == 45) {
                    // Send data from the both boards
                    lo_send(osc2, oscNamespace.c_str(), "iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii", touch.touch[0], touch.touch[1],touch.touch[2],
                    touch.touch[3],touch.touch[4],touch.touch[5], touch.touch[6], touch.touch[7], touch.touch[8],
                    touch.touch[9], touch.touch[10], touch.touch[11], touch.touch[12], touch.touch[13], touch.touch[14], touch.touch[15], touch.touch[16],touch.touch[17],
                    touch.touch[18],touch.touch[19],touch.touch[20], touch.touch[21], touch.touch[22], touch.touch[23],
                    touch.touch[24], touch.touch[25], touch.touch[26], touch.touch[27], touch.touch[28], touch.touch[29], touch2.touch[0], touch2.touch[1],touch2.touch[2],
                    touch2.touch[3],touch2.touch[4],touch2.touch[5], touch2.touch[6], touch2.touch[7], touch2.touch[8],
                    touch2.touch[9], touch2.touch[10], touch2.touch[11], touch2.touch[12], touch2.touch[13], touch2.touch[14]);
                } else if (TSTICK_SIZE == 60) {
                    // Send data from the first board
                    lo_send(osc2, oscNamespace.c_str(), "iiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii", touch.touch[0], touch.touch[1],touch.touch[2],
                    touch.touch[3],touch.touch[4],touch.touch[5], touch.touch[6], touch.touch[7], touch.touch[8],
                    touch.touch[9], touch.touch[10], touch.touch[11], touch.touch[12], touch.touch[13], touch.touch[14], touch.touch[15], touch.touch[16],touch.touch[17],
                    touch.touch[18],touch.touch[19],touch.touch[20], touch.touch[21], touch.touch[22], touch.touch[23],
                    touch.touch[24], touch.touch[25], touch.touch[26], touch.touch[27], touch.touch[28], touch.touch[29], touch2.touch[0], touch2.touch[1],touch2.touch[2],
                    touch2.touch[3],touch2.touch[4],touch2.touch[5], touch2.touch[6], touch2.touch[7], touch2.touch[8],
                    touch2.touch[9], touch2.touch[10], touch2.touch[11], touch2.touch[12], touch2.touch[13], touch2.touch[14], touch2.touch[15], touch2.touch[16],touch2.touch[17],
                    touch2.touch[18],touch2.touch[19],touch2.touch[20], touch2.touch[21], touch2.touch[22], touch2.touch[23],
                    touch2.touch[24], touch2.touch[25], touch2.touch[26], touch2.touch[27], touch2.touch[28], touch2.touch[29]);
                }
                else {
                    lo_send(osc2, oscNamespace.c_str(), "iiiiiiiiiiiiiii", touch.touch[0], touch.touch[1],touch.touch[2],
                        touch.touch[3],touch.touch[4],touch.touch[5], touch.touch[6], touch.touch[7], touch.touch[8],
                        touch.touch[9], touch.touch[10], touch.touch[11], touch.touch[12], touch.touch[13], touch.touch[14]);
                }
            #endif
            #ifdef touch_CAPSENSE
                lo_send(osc2, oscNamespace.c_str(), "iiiiiiiiiiiiiiii", capsense.data[0], capsense.data[1],capsense.data[2],
                    capsense.data[3],capsense.data[4],capsense.data[5], capsense.data[6], capsense.data[7], capsense.data[8],
                    capsense.data[9], capsense.data[10], capsense.data[11], capsense.data[12], capsense.data[13], capsense.data[14],
                    capsense.data[15]
            );
            #endif
    }

   // Set new callback and force next iteration
    OSCUpdate.setCallback(&oscInertialGestures);
    OSCUpdate.forceNextIteration();
}

void oscInertialGestures() {
  // Sending discrete OSC messages
    if (puara.IP1_ready()) {
       if (event.shake) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/shakexyz");
            lo_send(osc1, oscNamespace.c_str(), "fff", sensors.shake[0], sensors.shake[1], sensors.shake[2]);
        }
        if (event.jab) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/jabxyz");
            lo_send(osc1, oscNamespace.c_str(), "fff", sensors.jab[0], sensors.jab[1], sensors.jab[2]);
        }
    }
    if (puara.IP2_ready()) {
        if (event.shake) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/shakexyz");
            lo_send(osc2, oscNamespace.c_str(), "fff", sensors.shake[0], sensors.shake[1], sensors.shake[2]);
        }
        if (event.jab) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/jabxyz");
            lo_send(osc2, oscNamespace.c_str(), "fff", sensors.jab[0], sensors.jab[1], sensors.jab[2]);
        }
    }

    // Set new callback and force next iteration
    OSCUpdate.setCallback(&oscTouchGestures);
    OSCUpdate.forceNextIteration();
}

void oscTouchGestures() {
    // Sending touch gestures OSC
    if (puara.IP1_ready()) {
        if (event.brush) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/brush");
            lo_send(osc1, oscNamespace.c_str(), "f", sensors.brush);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/multibrush");
            lo_send(osc1, oscNamespace.c_str(), "fff", sensors.multibrush[0], sensors.multibrush[1], sensors.multibrush[2]);
        }
        if (event.rub) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/rub");
            lo_send(osc1, oscNamespace.c_str(), "f", sensors.rub);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/multirub");
            lo_send(osc1, oscNamespace.c_str(), "fff", sensors.multirub[0], sensors.multirub[1], sensors.multirub[2]);
        }
    }
    if (puara.IP2_ready()) {
        if (event.brush) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/brush");
            lo_send(osc2, oscNamespace.c_str(), "f", sensors.brush);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/multibrush");
            lo_send(osc2, oscNamespace.c_str(), "fff", sensors.multibrush[0], sensors.multibrush[1], sensors.multibrush[2]);
        }
        if (event.rub) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/rub");
            lo_send(osc2, oscNamespace.c_str(), "f", sensors.rub);
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/multirub");
            lo_send(osc2, oscNamespace.c_str(), "fff", sensors.multirub[0], sensors.multirub[1], sensors.multirub[2]);
        }
    }

    // Set new callback and force next iteration
    OSCUpdate.setCallback(&oscButtonGestures);
    OSCUpdate.forceNextIteration();
}

void oscButtonGestures() {
    // Sending button gestures OSC
    if (puara.IP1_ready()) {
        if (event.count) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/button/count");
            lo_send(osc1, oscNamespace.c_str(), "i", sensors.count);
        }
        if (event.tap) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/button/tap");
            lo_send(osc1, oscNamespace.c_str(), "i", sensors.tap);
        }
        if (event.dtap) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/button/dtap");
            lo_send(osc1, oscNamespace.c_str(), "i", sensors.dtap);
        }
        if (event.ttap) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/button/ttap");
            lo_send(osc1, oscNamespace.c_str(), "i", sensors.ttap);
        }
    }
    if (puara.IP2_ready()) {
        if (event.count) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/button/count");
            lo_send(osc2, oscNamespace.c_str(), "i", sensors.count);
        }
        if (event.tap) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/button/tap");
            lo_send(osc2, oscNamespace.c_str(), "i", sensors.tap);
        }
        if (event.dtap) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/button/dtap");
            lo_send(osc2, oscNamespace.c_str(), "i", sensors.dtap);
        }
        if (event.ttap) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "instrument/button/ttap");
            lo_send(osc2, oscNamespace.c_str(), "i", sensors.ttap);
        }
    }

    // Set new callback
    OSCUpdate.setCallback(&oscFSR);
}

void oscBattery() {
    // Sending battery data OSC
    if (puara.IP1_ready()) {
        if (event.battery) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "battery");
            lo_send(osc1, oscNamespace.c_str(), "i", sensors.battery);
        }
    }
    if (puara.IP2_ready()) {
        if (event.battery) {
            oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), "battery");
            lo_send(osc2, oscNamespace.c_str(), "i", sensors.battery);
        }
    }

    // Set new callback and loop back to first task
    OSCUpdate.setCallback(&getFuelGaugeData);
}
#endif



///////////
// setup //
///////////

void setup() {
    #ifdef Arduino_h
        Serial.begin(115200);
    #endif

    // Disable WiFi power save
    esp_wifi_set_ps(WIFI_PS_NONE);

    puara.set_version(firmware_version);
    puara.start();
    baseNamespace.append(puara.get_dmi_name());
    baseNamespace.append("/");
    oscNamespace = baseNamespace;

    #ifdef ARDUINO_LOLIN_D32_PRO // LED init for WEMOS boards
      ledcSetup(0, 5000, 8);
      ledcAttachPin(pin.led, 0);
    #endif

    std::cout << "    Initializing button configuration... ";
    if (button.initButton(pin.button)) {
        std::cout << "done" << "\n" 
                  << "Button Pin: " << button.getPin() << "\n" << std::endl;
    } else {
        std::cout << "initialization failed!" << std::endl;
    }

    std::cout << "    Initializing FSR... ";
    if (fsr.initFsr(pin.fsr, std::round(puara.getVarNumber("fsr_offset")))) {
        std::cout << "done (offset value: " << fsr.getOffset() << ")\n" 
                  << "FSR Pin: " << fsr.getPin() << "\n" << std::endl;
    } else {
        std::cout << "initialization failed!" << std::endl;
    }

    // Create sensorClass array
    std::vector<sensor> sensorClass;
    sensorClass.push_back(imu);
    sensorClass.push_back(touch);
    sensorClass.push_back(touch2);
    sensorClass.push_back(fuelgauge);

    //
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2CUPDATE_FREQ); // Fast mode plus

    // Initialising I2C Sensors
    if (sensormanager.initSensorManager(sensorClass)) {
        std::cout << "Initialised Sensor Manager" << std::endl;
    } else {
        std::cout << "Sensor Manager failed to initialise" << std::endl;
    }

    std::cout << "    Initializing Liblo server/client at " << puara.getLocalPORTStr() << " ... ";
    osc1 = lo_address_new(puara.getIP1().c_str(), puara.getPORT1Str().c_str());
    osc2 = lo_address_new(puara.getIP2().c_str(), puara.getPORT2Str().c_str());
    osc_server = lo_server_thread_new(puara.getLocalPORTStr().c_str(), error);
    lo_server_thread_add_method(osc_server, NULL, NULL, generic_handler, NULL);
    lo_server_thread_start(osc_server);
    std::cout << "done" << std::endl;

    std::cout << "    Initializing Libmapper device/signals... ";
    lm_dev = mpr_dev_new(puara.get_dmi_name().c_str(), 0);
    lm.fsr = mpr_sig_new(lm_dev, MPR_DIR_OUT, "raw/fsr", 1, MPR_FLT, "un", &lm.fsrMin, &lm.fsrMax, 0, 0, 0);
    lm.accel = mpr_sig_new(lm_dev, MPR_DIR_OUT, "raw/accel", 3, MPR_FLT, "m/s^2",  &lm.accelMin, &lm.accelMax, 0, 0, 0);
    lm.gyro = mpr_sig_new(lm_dev, MPR_DIR_OUT, "raw/gyro", 3, MPR_FLT, "rad/s", &lm.gyroMin, &lm.gyroMax, 0, 0, 0);
    lm.magn = mpr_sig_new(lm_dev, MPR_DIR_OUT, "raw/mag", 3, MPR_FLT, "uTesla", &lm.magnMin, &lm.magnMax, 0, 0, 0);
    lm.quat = mpr_sig_new(lm_dev, MPR_DIR_OUT, "orientation", 4, MPR_FLT, "qt", lm.quatMin, lm.quatMax, 0, 0, 0);
    lm.ypr = mpr_sig_new(lm_dev, MPR_DIR_OUT, "ypr", 3, MPR_FLT, "fl", lm.yprMin, lm.yprMax, 0, 0, 0);
    lm.shake = mpr_sig_new(lm_dev, MPR_DIR_OUT, "instrument/shake", 3, MPR_FLT, "fl", lm.shakeMin, lm.shakeMax, 0, 0, 0);
    lm.jab = mpr_sig_new(lm_dev, MPR_DIR_OUT, "instrument/jab", 3, MPR_FLT, "fl", lm.jabMin, lm.jabMax, 0, 0, 0);
    lm.brush = mpr_sig_new(lm_dev, MPR_DIR_OUT, "instrument/brush", 1, MPR_FLT, "un", lm.brushMin, lm.brushMax, 0, 0, 0);
    lm.rub = mpr_sig_new(lm_dev, MPR_DIR_OUT, "instrument/rub", 1, MPR_FLT, "un", lm.rubMin, lm.rubMax, 0, 0, 0);
    lm.multibrush = mpr_sig_new(lm_dev, MPR_DIR_OUT, "instrument/multibrush", 4, MPR_FLT, "un", lm.brushMin, lm.brushMax, 0, 0, 0);
    lm.multirub = mpr_sig_new(lm_dev, MPR_DIR_OUT, "instrument/multirub", 4, MPR_FLT, "un", lm.rubMin, lm.rubMax, 0, 0, 0);
    #ifdef touch_TRILL
        lm.touch = mpr_sig_new(lm_dev, MPR_DIR_OUT, "raw/capsense", touch.touchSize, MPR_INT32, "un", &lm.touchMin, &lm.touchMax, 0, 0, 0);
    #endif
    #ifdef touch_CAPSENSE
        lm.touch = mpr_sig_new(lm_dev, MPR_DIR_OUT, "raw/capsense", capsense.touchStripsSize, MPR_INT32, "un", &lm.touchMin, &lm.touchMax, 0, 0, 0);
    #endif
    lm.count = mpr_sig_new(lm_dev, MPR_DIR_OUT, "instrument/count", 1, MPR_INT32, "un", &lm.countMin, &lm.countMax, 0, 0, 0);
    lm.tap = mpr_sig_new(lm_dev, MPR_DIR_OUT, "instrument/tap", 1, MPR_INT32, "un", &lm.tapMin, &lm.tapMax, 0, 0, 0);
    lm.ttap = mpr_sig_new(lm_dev, MPR_DIR_OUT, "instrument/triple tap", 1, MPR_INT32, "un", &lm.tapMin, &lm.tapMax, 0, 0, 0);
    lm.dtap = mpr_sig_new(lm_dev, MPR_DIR_OUT, "instrument/double tap", 1, MPR_INT32, "un", &lm.tapMin, &lm.tapMax, 0, 0, 0);
    lm.bat = mpr_sig_new(lm_dev, MPR_DIR_OUT, "battery", 1, MPR_FLT, "percent", &lm.batMin, &lm.batMax, 0, 0, 0);
    std::cout << "done" << std::endl;

    // Setting Deep sleep wake button
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_15,0); // 1 = High, 0 = Low
    
    // Using Serial.print and delay to prevent interruptions
    delay(500);
    Serial.println(); 
    Serial.println(puara.get_dmi_name().c_str());
    Serial.println("Edu Meneses\nMetalab - Société des Arts Technologiques (SAT)\nIDMIL - CIRMMT - McGill University");
    Serial.print("Firmware version: "); Serial.println(firmware_version); Serial.println("\n");

    // enable all tasks
    runnerTstick.enableAll();
}

//////////
// loop //
//////////

void loop() {
    // Read Sensors
    runnerTstick.execute();
    // updateSensors();
}

