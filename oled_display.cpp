#include "oled_display.h"
#include "Arduino.h"
#include "math.h"

OledDisplay::OledDisplay(SSD1306 *display) {
    _display = display;
}

void OledDisplay::init() {
    _display->init();
    _display->flipScreenVertically();
    _display->setFont(ArialMT_Plain_10);
}

void OledDisplay::loop() {
    page();
}

void OledDisplay::setPage(uint8_t page) {
    _page = page;
}

void OledDisplay::page() {

    static uint32_t lastUpdate = 0;

    _forceDisplay = false;
    switch (_page) {
        
        case OLED_PAGE_BEACON_STATUS:
            renderPageStatus();
            break;
    }

    lastUpdate = millis();
}

void OledDisplay::renderPageStatus() {

    _display->clear();

    String val;

    _display->setFont(ArialMT_Plain_10);
    
    //Gyro calibration
    if (imu.gyroCalibration.state == CALIBRATION_DONE) {
        val = "Gyro: OK";
    } else {
        val = "Gyro: Cal";
    }
    _display->drawString(0, 0, val);

    //Gyro calibration
    if (
        thumbJoystick.calibration.state == CALIBRATION_DONE &&
        thumbJoystick.max[AXIS_X] > 750 && 
        thumbJoystick.max[AXIS_Y] > 750 &&
        thumbJoystick.min[AXIS_X] < -750 && 
        thumbJoystick.min[AXIS_Y] < -750
    ) {
        val = "Stick: OK";
    } else {
        val = "Stick: Cal";
    }
    _display->drawString(64, 0, val);

    _display->setFont(ArialMT_Plain_24);
    if (device.getActionEnabled()) {
        _display->drawString(46, 16, "Hot");

        _display->setFont(ArialMT_Plain_10);
        _display->drawString(0, 42, "R:" + String(fscalef(output.channels[ROLL] - 1500, -500, 500, -100, 100), 0) + "%");
        _display->drawString(0, 52, "T:" + String(fscalef(output.channels[THROTTLE] - 1500, -500, 500, -100, 100), 0) + "%");
        _display->drawString(64, 42, "P:" + String(fscalef(output.channels[PITCH] - 1500, -500, 500, -100, 100), 0) + "%");
        _display->drawString(64, 52, "Y:" + String(fscalef(output.channels[YAW] - 1500, -500, 500, -100, 100), 0) + "%");

    } else {
        _display->drawString(36, 16, "Safe");
    }

    _display->display();
}