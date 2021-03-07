#include "oled_display.h"
#include "Arduino.h"

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

    //Do not allow for OLED to be updated too often
    if (lastUpdate > 0 && millis() - lastUpdate < 200 && _forceDisplay == false) {
        return;
    }

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
    if (imu.gyroCalibration.state == CALIBRATION_DONE) {
        val = "Gyro: OK";
    } else {
        val = "Gyro: Cal";
    }
    _display->drawString(0, 0, val);

    _display->setFont(ArialMT_Plain_16);
    _display->drawString(0, 32, "OK");

    _display->display();
}