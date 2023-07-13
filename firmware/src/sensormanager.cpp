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

int sensorManager::initSensorManager(std::vector<sensor> sensorClass) {
    config_spiffs();    
    read_json();
    for (auto &sensor : sensorMap) {
        if (sensor.second.enabled) {
            // Add sensor class
            sensor.second.sensorObject = sensorClass[sensor.second.classIdx];
            // Add to inactive I2C list
            updateInactiveList(sensor.second);
        }
    }   
    return 1;
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

    std::cout << "json: Opening config json file" << std::endl;
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
    // inspired from puara read_config_json_internal
    std::cout << "json: Getting data" << std::endl;
    cJSON *root = cJSON_Parse(contents.c_str());
    cJSON *item = cJSON_GetObjectItem(root,"items");
    int i;
    // loop through each item
    for (i = 0 ; i < cJSON_GetArraySize(item) ; i++) {
        // create temperary struture
        sensorManager::sensorInfo tmp;
        // Fill with data
        cJSON * subitem = cJSON_GetArrayItem(item, i);
        tmp.name = cJSON_GetObjectItem(subitem, "name")->valuestring;
        tmp.commType = cJSON_GetObjectItem(subitem, "commType")->valuestring;
        tmp.address =  cJSON_GetObjectItem(subitem, "address")->valueint;
        tmp.active = cJSON_GetObjectItem(subitem, "isActive")->valueint;
        tmp.enabled = cJSON_GetObjectItem(subitem, "isEnabled")->valueint;
        tmp.classIdx = cJSON_GetObjectItem(subitem, "classIdx")->valueint;

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
    }
    cJSON_Delete(root);
}

void sensorManager::scanInactiveI2C() {
    byte error, address;
    int nDevices;
    std::cout << "Scanning I2C devices..." << std::endl;
    nDevices = 0;
    Wire.begin();
    for (auto it = inactiveI2C.begin(); it != inactiveI2C.end();)
    {
        address = *it;
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            sensorMap[address].active = true;
            inactiveI2C.erase(it);
            std::cout 
            << "Sensor " << sensorMap[address].name << " has been found" << "\n"
            << std::endl;
            nDevices++;
        }
        else if (error==4)
        {
            ++it;
            std::cout 
            << "Sensor " << sensorMap[address].name << " did not respond." << "\n"
            << "Unknown error at address 0x" << address << "\n" 
            << std::endl;
        }    
    }
    if (nDevices == 0)
        std::cout << "No I2C devices found\n" << address << std::endl;
    else
        std::cout << "done\n" << address << std::endl;
}

void sensorManager::scanActiveI2C() {
    byte error, address;
    int nDevices;
    std::cout << "Scanning I2C devices..." << std::endl;
    nDevices = 0;
    Wire.begin();
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
        }
        else if (error==4)
        {
            sensorMap[address].active = false;
            activeI2C.erase(it);
            std::cout 
            << "Sensor " << sensorMap[address].name << " did not respond." << "\n"
            << "Unknown error at address 0x" << address << "\n" 
            << std::endl;
        }    
    }
}

int sensorManager::initSensors() {
    for (auto &sensor : sensorMap) {
        if (sensor.second.enabled && sensor.second.active) {
            //Initialise sensor
            if (sensor.second.sensorObject.init(sensor.second.address)) {
                std::cout 
                << "Sensor " << sensor.second.name << "initialised successfuly" << "\n"
                << std::endl;
            } else {
                std::cout 
                << "Sensor " << sensor.second.name << "failed to initialised" << "\n"
                << std::endl;
                // update inactive sensor list
                sensor.second.active = false;
                updateInactiveList(sensor.second);
            }
        }
    }
}

int sensorManager::getSensorData() {
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
}

void sensorManager::updateInactiveList(sensorManager::sensorInfo inactiveSensor) {
    if (inactiveSensor.commType == "I2C") {
        inactiveI2C.push_back(inactiveSensor.address);
    } else if (inactiveSensor.commType == "GPIO") {
        inactiveGPIO.push_back(inactiveSensor.address);
    } else {
        std::cout 
        << "\nInvalid Communication type " << inactiveSensor.commType << " for sensor " << inactiveSensor.name << "\n"
        << "Communication type must be either I2C or GPIO" << "\n"
        << "Sensor will not be added to sensorManager" << "\n"
        << std::endl;
    }
}