// C++ class for updating liblo messages and bundles
#ifndef _LIBLO_OSC_BUNDLE_H_
#define _LIBLO_OSC_BUNDLE_H_
#include <string>
#include <lo/config.h>
#include <lo/lo_types_internal.h>
#include <lo/lo.h>
#include <lo/lo_lowlevel.h>

struct message_config {
    lo_type msg_type;
    size_t num_elements;
    void * data;    

    message_config(float *arr, size_t num) : msg_type(LO_FLOAT), num_elements(num), data(NULL) {
        data = malloc(sizeof(arr));
        memcpy(data, &arr, sizeof(arr));
    }
    message_config(int *arr, size_t num) : msg_type(LO_INT32), num_elements(num), data(NULL) {
        data = malloc(sizeof(arr));
        memcpy(data, &arr, sizeof(arr));
    }
};

// Bundle
class oscBundle {
    public:
        // Liblo bundles
        lo_bundle bundle;

        // Properties
        int num_messages = 0;
        std::string baseNamespace = "/";
        std::string oscNamespace;
    
        // Methods
        int init(const char *baseName);
        int update_message(int i, size_t size, float *data);
        int update_message(int i, size_t size, int *data);
        int update_message(int i, int data);
        int update_message(int i, float data);
        void update(message_config *msg, size_t num_msg);

        // Add data
        void add(const char *path, int value);
        void add(const char *path, float value);
        void add_array(const char *path, int size, int *value);
        void add_array(const char *path, int size,  float *value);
};

#endif