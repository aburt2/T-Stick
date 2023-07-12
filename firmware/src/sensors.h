// Create library for managing sensors

#ifndef SENSOR_H
#define SENSOR_H
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
typedef int (*FnPtr)(int, int);

class sensorManager {
    public:
        int initSensorManager();
        void scanInactiveI2C();
        void scanActiveI2C();
        int initSensors(std::vector<FnPtr> initFunctions);
        uint8_t getSensorData(std::vector<FnPtr> pollFunctions);
        void changeSensorStatus();
        void updateScanInterval();


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
    private:
        int scanInterval = 1;
        struct sensorInfo {
            std::string name;
            std::string commType;
            uint8_t address;
            bool active;
            bool enabled;
            int initidx;
            int pollidx;
        };
        static std::vector<sensorInfo> sensors;
        static std::vector<uint8_t> inactiveI2C;
        static std::vector<uint8_t> activeI2C;
        static std::vector<uint8_t> inactiveGPIO;
        static std::vector<uint8_t> activeGPIO;  
        static std::map<uint8_t, sensorInfo> sensorMap;      
};

#endif