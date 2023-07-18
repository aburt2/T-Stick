// Create library for managing sensors

#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H
#include <esp_spiffs.h>
#include <cJSON.h>
#include <stdio.h>
#include <string>
#include <cstring>
#include <ostream>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <unordered_map>
#include <esp_spiffs.h>
#include <vector>
#include <map>

#include <Arduino.h>
#include <Wire.h>
#include <sensor.h>

class sensorManager {
    public:
        // Sensor information structure
        struct sensorInfo {
            std::string name;
            std::string sensorType;
            std::string commType;
            int address;
            bool active;
            bool enabled;
            int classIdx;
            int numFailure;
            sensor sensorObject;
        };
        // Initialisation Functions
        int initSensorManager(std::vector<sensor> sensorClass);
        int initSensors();

        // JSON reading functions
        static void read_json();
        // static void write_json();
        static void read_json_internal(std::string& contents);
        
        // Spiffs related methods
        static void config_spiffs();
        static void mount_spiffs();
        static void unmount_spiffs();
        static esp_vfs_spiffs_conf_t spiffs_config;
        static std::string spiffs_base_path;
        static const uint8_t spiffs_max_files = 10;
        static const bool spiffs_format_if_mount_failed = false;

        // I2C Scanning functions
        void scanInactiveI2C();
        void scanActiveI2C();
        void updateInactiveList(sensorInfo);
        
        // Sensor data functions
        int getSensorData();
        void changeSensorStatus();
        void updateScanInterval();

        // Return sensor info
        bool checkSensorStatus(std::string sensorName);
        sensor getSensorObject(std::string sensorName);

        // Max failure
        int maxFailures = 5;
    private:
        int scanInterval = 1;
        static std::vector<sensorInfo> sensors;
        static std::vector<uint8_t> inactiveI2C;
        static std::vector<uint8_t> activeI2C;
        static std::vector<uint8_t> inactiveGPIO;
        static std::vector<uint8_t> activeGPIO;  
        static std::map<uint8_t, sensorInfo> sensorMap;   
        static std::map<std::string, uint8_t> nameMap;      
};

#endif