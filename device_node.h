#pragma once

#ifndef DEVICE_NODE_H
#define DEVICE_NODE_H

#include <Arduino.h>
#include "math.h"
#include "types.h"

extern dataOutput_t output;

enum deviceMode_e {
    DEVICE_MODE_MOTION_CONTROLLER = 0,
    DEVICE_MODE_LAST
};

class DeviceNode {
    public:
        DeviceNode(void);
        void begin(void);
        deviceMode_e getDeviceMode(void);
        void setDeviceMode(deviceMode_e mode);
        bool getActionEnabled(void);
        void setActionEnabled(bool enabled);
    private:
        deviceMode_e _currentDeviceMode = DEVICE_MODE_MOTION_CONTROLLER;
        deviceMode_e _previousDeviceMode = DEVICE_MODE_MOTION_CONTROLLER;
        bool _actionEnabled = false;
        bool _previousActionEnabled = false;
};

#endif