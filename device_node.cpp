#include "device_node.h"

DeviceNode::DeviceNode(void) {
}

void DeviceNode::begin(void) {
    
}

deviceMode_e DeviceNode::getDeviceMode(void) {
    return _currentDeviceMode;
}

void DeviceNode::setDeviceMode(deviceMode_e mode) {
    _previousDeviceMode = _currentDeviceMode;
    _currentDeviceMode = mode;
}

bool DeviceNode::getActionEnabled(void) {
    return _actionEnabled;
}

void DeviceNode::setActionEnabled(bool enabled) {
    _previousActionEnabled = _actionEnabled;
    _actionEnabled = enabled;
}