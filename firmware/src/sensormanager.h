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
        int initSensorManager(std::vector<sensor> sensorClass);
        void scanInactiveI2C();
        void scanActiveI2C();
        int initSensors();
        int getSensorData();
        void changeSensorStatus();
        void updateScanInterval();
        // Sensor info
        struct sensorInfo {
            std::string name;
            std::string sensorType;
            std::string commType;
            uint8_t address;
            bool active;
            bool enabled;
            int classIdx;
            sensor sensorObject;
        };
        void updateInactiveList(sensorInfo);

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
        static std::vector<sensorInfo> sensors;
        static std::vector<uint8_t> inactiveI2C;
        static std::vector<uint8_t> activeI2C;
        static std::vector<uint8_t> inactiveGPIO;
        static std::vector<uint8_t> activeGPIO;  
        static std::map<uint8_t, sensorInfo> sensorMap;      
};

#endif