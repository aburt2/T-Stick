// Include sensor manager
# include "sensormanager.h"

//Define static members
esp_vfs_spiffs_conf_t sensorManager::spiffs_config;
std::vector<sensorManager::sensorInfo> sensorManager::sensors;
std::string sensorManager::spiffs_base_path;
std::vector<uint8_t> sensorManager::inactiveI2C;
std::vector<uint8_t> sensorManager::activeI2C;
std::vector<uint8_t> sensorManager::inactiveGPIO;
std::vector<uint8_t> sensorManager::activeGPIO;  
std::map<uint8_t, sensorManager::sensorInfo> sensorManager::sensorMap;
std::map<std::string, uint8_t> sensorManager::nameMap;

int sensorManager::initSensorManager(std::vector<sensor> sensorClass) {
    //Read Config data
    config_spiffs();    
    read_json();

    //Set up inactive list
    for (auto &sensorPair : sensorMap) {
        if (sensorPair.second.enabled) {
            // Add sensor class
            sensorPair.second.sensorObject = sensorClass[sensorPair.second.classIdx];
            std::cout 
            << "Added Sensor " << sensorPair.second.name << " class object." << "\n"
            << std::endl;
            // Add to inactive I2C list
            sensorManager::updateInactiveList(sensorPair.second);
        }
    }
    //Initialise Sensors
    status = sensorManager::initSensors();

    // Return status
    return status;
}

void sensorManager::config_spiffs() {
    spiffs_base_path = "/spiffs";
}

void sensorManager::mount_spiffs() {
    // copied from puara
    if (!esp_spiffs_mounted(spiffs_config.partition_label)) {
        std::cout << "spiffs: Initializing SPIFFS" << std::endl;

        spiffs_config.base_path = sensorManager::spiffs_base_path.c_str();
        spiffs_config.max_files = sensorManager::spiffs_max_files;
        spiffs_config.partition_label = NULL;
        spiffs_config.format_if_mount_failed = sensorManager::spiffs_format_if_mount_failed;

        // Use settings defined above to initialize and mount SPIFFS filesystem.
        // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
        esp_err_t ret = esp_vfs_spiffs_register(&spiffs_config);

        if (ret != ESP_OK) {
            if (ret == ESP_FAIL) {
                std::cout << "spiffs: Failed to mount or format filesystem" << std::endl;
            } else if (ret == ESP_ERR_NOT_FOUND) {
                std::cout << "spiffs: Failed to find SPIFFS partition" << std::endl;
            } else {
                std::cout << "spiffs: Failed to initialize SPIFFS (" << esp_err_to_name(ret) << ")"  << std::endl;
            }
            return;
        }

        size_t total = 0, used = 0;
        ret = esp_spiffs_info(spiffs_config.partition_label, &total, &used);
        if (ret != ESP_OK) {
            std::cout << "spiffs: Failed to get SPIFFS partition information (" << esp_err_to_name(ret) << ")"  << std::endl;
        } else {
            std::cout << "spiffs: Partition size: total: " << total << ", used: " << used  << std::endl;
        }
    } else {
        std::cout << "spiffs: SPIFFS already initialized" << std::endl;
    }
}

void sensorManager::unmount_spiffs() {
    // copied from puara
    // All done, unmount partition and disable SPIFFS
    if (esp_spiffs_mounted(spiffs_config.partition_label)) {
        esp_vfs_spiffs_unregister(spiffs_config.partition_label);
        std::cout << "spiffs: SPIFFS unmounted" << std::endl;
    } else {
        std::cout << "spiffs: SPIFFS not found" << std::endl;
    }
}

void sensorManager::read_json() { // Deserialize
    // copied from puara read_config_json
    std::cout << "json: Mounting FS" << std::endl;
    sensorManager::mount_spiffs();

    std::cout << "json: Opening sensor config json file" << std::endl;
    FILE* f = fopen("/spiffs/sensordefs.json", "r");
    if (f == NULL) {
        std::cout << "json: Failed to open file" << std::endl;
        return;
    }

    std::cout << "json: Reading json file" << std::endl;
    std::ifstream in("/spiffs/sensordefs.json");
    std::string contents((std::istreambuf_iterator<char>(in)), 
    std::istreambuf_iterator<char>());

    sensorManager::read_json_internal(contents);

    fclose(f);
    sensorManager::unmount_spiffs();
}

void sensorManager::read_json_internal(std::string& contents) {
    // counter for sensors
    int nSensors;
    // inspired from puara read_config_json_internal
    std::cout << "json: Getting data" << std::endl;
    cJSON *root = cJSON_Parse(contents.c_str());
    cJSON *setting = NULL;
    cJSON *settings = NULL;
    
    std::cout << "json: Parse sensor settings information" << std::endl;
    settings = cJSON_GetObjectItemCaseSensitive(root, "sensorsettings");

    // loop through each item
    std::cout << "json: Extract Sensor info" << std::endl;
    cJSON_ArrayForEach(setting, settings) {
        // create temperary struture
        sensorManager::sensorInfo tmp;

        // Fill with data from json
        tmp.name = cJSON_GetObjectItem(setting, "name")->valuestring;
        tmp.commType = cJSON_GetObjectItem(setting, "commType")->valuestring;
        tmp.address =  cJSON_GetObjectItem(setting, "address")->valueint;
        tmp.active = cJSON_GetObjectItem(setting, "isActive")->valuedouble;
        tmp.enabled = cJSON_GetObjectItem(setting, "isEnabled")->valuedouble;
        tmp.classIdx = cJSON_GetObjectItem(setting, "classIdx")->valuedouble;

        // Reset their number of failures
        tmp.numFailure = -1;

        // Print collected sensor data
        std::cout << "\njson: Sensor added:\n\n"
        << "name: " << tmp.name << "\n"
        << "commType: " << tmp.commType << "\n"
        << "address: " << tmp.address << "\n"
        << "isActive: " << tmp.active << "\n"
        << "isEnabled: " << tmp.enabled << "\n"
        << std::endl;

        // Add to sensor array
        sensors.push_back(tmp);
        sensorMap[tmp.address] = tmp;
        nameMap[tmp.name] = tmp.address;

        //Increment Sensor counter
        nSensors++;
    }
    cJSON_Delete(root);

    // If no sensors were initialised flag user 
    if (nSensors == 0) {
        std::cout << "No sensors initialised, check sensor config" << "\n" << std::endl;
    }
}

void sensorManager::scanInactiveI2C() {
    byte error, address;
    int nDevices;
    std::cout << "Scanning Inactive I2C devices..." << std::endl;
    nDevices = 0;
    for (auto it = inactiveI2C.begin(); it != inactiveI2C.end();)
    {
        address = *it;
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        // std::cout 
        // << "Looking for sensor  " << sensorMap[address].name << "\n"
        // << std::endl;
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            sensorMap[address].active = true;
            inactiveI2C.erase(it);
            std::cout << "Sensor " << sensorMap[address].name << " has been found" << std::endl;
            nDevices++;
        }
        else
        {
            sensorMap[address].numFailure++;
            if (sensorMap[address].numFailure == 0) {
                std::cout 
                << "Sensor " << sensorMap[address].name << " did not respond." << "\n"
                << "Unknown error at address 0x" << int(address) << "\n"
                << std::endl;
                ++it;
            } else if (sensorMap[address].numFailure == maxFailures) {
                sensorMap[address].enabled = false;
                std::cout 
                << "Disabling Sensor " << sensorMap[address].name << "\n"
                << "Sensor failed to respond " << maxFailures << " times.\n"
                << std::endl;
                inactiveI2C.erase(it);
            } else {
                ++it;
                std::cout << "Sensor " << sensorMap[address].name << " did not respond." << std::endl;
            }
        }    
    }
    if (nDevices == 0)
        std::cout << "No I2C devices found" << std::endl;
    else
        std::cout << "done" << address << std::endl;
}

void sensorManager::scanActiveI2C() {
    byte error, address;
    int nDevices;
    std::cout << "Scanning Active Sensor I2C devices..." << std::endl;
    nDevices = 0;
    for (auto it = activeI2C.begin(); it != activeI2C.end();)
    {
        address = *it;
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            nDevices++;
            ++it;
            std::cout << "Sensor " << sensorMap[address].name << " responded." << std::endl;
        }
        else
        {
            sensorMap[address].active = false;
            activeI2C.erase(it);
            std::cout 
            << "Sensor " << sensorMap[address].name << " did not respond." << "\n"
            << "Unknown error at address 0x" << address << "\n" 
            << std::endl;
        }    
    }
    if (nDevices == 0) {
        std::cout << "No I2C devices found\n" << std::endl;
    } else {
        std::cout << "done\n" << std::endl;
    }
}

int sensorManager::initSensors() {
    // Check there are any sensors at all
    if (sensorManager::getnumSensors() == 0) {
        // No sensors are in the sensor map
        return 0;
    }
    // Scan inactive sensors
    sensorManager::scanInactiveI2C();

    // Initialise Detected Sensors
    for (auto & sensorPair : sensorMap) {
        if (sensorPair.second.enabled && sensorPair.second.active) {
            //Initialise sensor
            if (sensorPair.second.sensorObject.init(sensorPair.second.address)) {
                std::cout 
                << "Sensor " << sensorPair.second.name << "initialised successfuly" << "\n"
                << std::endl;
            } else {
                std::cout 
                << "Sensor " << sensorPair.second.name << "failed to initialised" << "\n"
                << std::endl;
                // update inactive sensor list
                sensorPair.second.active = false;
                sensorManager::updateInactiveList(sensorPair.second);
            }
        }
    }

    // Return 1 on completion
    return 1;
}

int sensorManager::getSensorData() {
    // Get data from each sensor
    for (auto &sensor : sensorMap) {
        if (sensor.second.enabled && sensor.second.active) {
            //Initialise sensor
            if (sensor.second.sensorObject.readData()) {
            } else {
                std::cout 
                << "Failed to get data for sensor " << sensor.second.name << "\n"
                << std::endl;
                // update inactive sensor list
                sensor.second.active = false;
                updateInactiveList(sensor.second);
            }
        }
    }
    // return 1 for completion
    return 1;
}

void sensorManager::updateInactiveList(sensorManager::sensorInfo inactiveSensor) {
    std::string commtype ("I2C");
    if (inactiveSensor.commType == commtype) {
        inactiveI2C.push_back(inactiveSensor.address);
        std::cout 
        << "Sensor " << inactiveSensor.name << " added to inactive list" << "\n"
        << std::endl;
    } else {
        std::cout 
        << "\nInvalid Communication type " << inactiveSensor.commType << " for sensor " << inactiveSensor.name << "\n"
        << "Communication type must be either I2C" << "\n"
        << "Sensor will not be added to sensorManager" << "\n"
        << std::endl;
    }
}

bool sensorManager::checkSensorStatus(std::string sensorName)  {
    // Check if a sensor is active
    uint8_t i2caddress = nameMap[sensorName];
    if (sensorMap[i2caddress].active) {
        return true;
    } else {
        return false;
    }
}

sensor sensorManager::getSensorObject(std::string sensorName) {
    // Get the i2c address of the sensor
    uint8_t i2caddress = nameMap[sensorName];

    // Return the sensor object
    return sensorMap[i2caddress].sensorObject;
}

int sensorManager::getnumActiveSensors() {
    // Get number of active sensors
    return activeI2C.size();
}

int sensorManager::getnumInactiveSensors() {
    // Get number of active sensors
    return inactiveI2C.size();
}

int sensorManager::getnumSensors() {
    // Get number of active sensors
    return sensorMap.size();
}