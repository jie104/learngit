#include "imu_receive.hpp"
extern "C" {
#include "protocol_wrapper.h"
};

imu_data_handler g_imuCallback;
int imuDataCallback(void *data, u16 length, u16 type) {
    if (g_imuCallback) {
        g_imuCallback(data, length, type);
    }
    return 0;
}

int imuOutputInformation(const char *data) {
    LOG(INFO)<<data;
    return 0;
}