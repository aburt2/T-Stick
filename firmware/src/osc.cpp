#include "osc.hpp"

int oscBundle::init(const char *baseName) {
    baseNamespace.append(baseName);
    baseNamespace.append("/");
    oscNamespace = baseNamespace;

    // Create bundle
    bundle = lo_bundle_new(LO_TT_IMMEDIATE);

    if (!bundle) {
        return 1; // return 1 on failure
    }
}

int oscBundle::update_message(int i, size_t size, float *data) {
    // Update specific message
    if (i >= num_messages) {
        return 1; // return 1 on failure
    }   

    // Update the message
    memcpy((char *)bundle->elmnts[i].content.message.msg->data, data, sizeof(float)*size);
    return 0;
}

int oscBundle::update_message(int i, size_t size, int *data) {
    // Update specific message
    if (i >= num_messages) {
        return 1; // return 1 on failure
    }   

    // Update the message
    memcpy((char *)bundle->elmnts[i].content.message.msg->data, data, sizeof(int)*size);
    return 0;
}

int oscBundle::update_message(int i, int data) {
    // Update specific message
    if (i >= num_messages) {
        return 1; // return 1 on failure
    }   

    // Update the message
    memcpy((char *)bundle->elmnts[i].content.message.msg->data, &data, sizeof(int));
    return 0;
}

int oscBundle::update_message(int i, float data) {
    // Update specific message
    if (i >= num_messages) {
        return 1; // return 1 on failure
    }   

    // Update the message
    memcpy((char *)bundle->elmnts[i].content.message.msg->data, &data, sizeof(float));
    return 0;
}

void oscBundle::add(const char *path, int value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_int32(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    ret = lo_bundle_add_message(bundle, oscNamespace.c_str(), tmp_osc);
    num_messages++;
}
void oscBundle::add(const char *path, float value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    int ret = 0;
    lo_message tmp_osc = lo_message_new();
    ret = lo_message_add_float(tmp_osc, value);
    if (ret < 0) {
        lo_message_free(tmp_osc);
        return;
    }
    ret = lo_bundle_add_message(bundle, oscNamespace.c_str(), tmp_osc);
    num_messages++;
}
void oscBundle::add_array(const char *path, int size, int *value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    lo_message tmp_osc = lo_message_new();
    for (int i = 0; i < size; i++) {
        lo_message_add_int32(tmp_osc, value[i]);
    }
    lo_bundle_add_message(bundle, oscNamespace.c_str(), tmp_osc);
    num_messages++;
}

void oscBundle::add_array(const char *path, int size,  float *value) {
    oscNamespace.replace(oscNamespace.begin()+baseNamespace.size(),oscNamespace.end(), path);
    lo_message tmp_osc = lo_message_new();
    for (int i = 0; i < size; i++) {
        lo_message_add_float(tmp_osc, value[i]);
    }
    lo_bundle_add_message(bundle, oscNamespace.c_str(), tmp_osc);
    num_messages++;
}

