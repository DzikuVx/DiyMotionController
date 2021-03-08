#ifndef OLED_DISPLAY
#define OLED_DISPLAY

#include "SSD1306.h"
#include "math.h"
#include "types.h"
#include "device_node.h"

extern imuData_t imu;
extern thumb_joystick_t thumbJoystick;
extern DeviceNode device;

enum txOledPages {
    OLED_PAGE_NONE,
    OLED_PAGE_BEACON_STATUS,
};

#define OLED_COL_COUNT 64
#define OLED_DISPLAY_PAGE_COUNT 2

class OledDisplay {
    public:
        OledDisplay(SSD1306 *display);
        void init();
        void loop();
        void setPage(uint8_t page);
    private:
        SSD1306 *_display;
        void renderPageStatus();
        void page();
        uint8_t _page = OLED_PAGE_NONE;
        bool _forceDisplay = false;
};


#endif