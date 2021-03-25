#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include "QmuTactile.h"
#include "sbus.h"
#include "math.h"
#include "types.h"
#include "SSD1306.h"
#include "oled_display.h"
#include "device_node.h"

/*
 * Choose Trainer output type. Uncommend correcty line
 */
// #define TRAINER_MODE_SBUS
#define TRAINER_MODE_PPM

#define MPU6050_UPDATE_TASK_MS 25
#define OUTPUT_UPDATE_TASK_MS 20
#define SERIAL_TASK_MS 50
#define SERIAL1_RX 25
#define SERIAL1_TX 14
#define I2C1_SDA_PIN 21
#define I2C1_SCL_PIN 22

#define I2C2_SDA_PIN 0
#define I2C2_SCL_PIN 23
 
TwoWire I2C1 = TwoWire(0); //OLED bus
TwoWire I2C2 = TwoWire(1); //Gyro bus

#define PIN_BUTTON_THUMB 4
#define PIN_GPS_RX 2
#define PIN_BUTTON_TRIGGER 15

#define PIN_THUMB_JOYSTICK_X 13
#define PIN_THUMB_JOYSTICK_Y 33
#define PIN_THUMB_JOYSTICK_SW 32

Adafruit_MPU6050 mpu;
sensors_event_t acc, gyro, temp;

uint32_t nextSerialTaskMs = 0;

imuData_t imu;
dataOutput_t output;
thumb_joystick_t thumbJoystick;
DeviceNode device;

#ifdef TRAINER_MODE_SBUS
#define SBUS_UPDATE_TASK_MS 15
uint8_t sbusPacket[SBUS_PACKET_LENGTH] = {0};
HardwareSerial sbusSerial(1);
uint32_t nextSbusTaskMs = 0;
#endif

#ifdef TRAINER_MODE_PPM

#define PPM_FRAME_LENGTH 22500
#define PPM_PULSE_LENGTH 300
#define PPM_CHANNELS 8

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

enum ppmState_e {
    PPM_STATE_IDLE,
    PPM_STATE_PULSE,
    PPM_STATE_FILL,
    PPM_STATE_SYNC
};

void IRAM_ATTR onPpmTimer() {

    static uint8_t ppmState = PPM_STATE_IDLE;
    static uint8_t ppmChannel = 0;
    static uint8_t ppmOutput = LOW;
    static int usedFrameLength = 0;
    int currentChannelValue;

    portENTER_CRITICAL(&timerMux);

    if (ppmState == PPM_STATE_IDLE) {
        ppmState = PPM_STATE_PULSE;
        ppmChannel = 0;
        usedFrameLength = 0;
    }

    if (ppmState == PPM_STATE_PULSE) {
        ppmOutput = HIGH;
        usedFrameLength += PPM_PULSE_LENGTH;
        ppmState = PPM_STATE_FILL;

        timerAlarmWrite(timer, PPM_PULSE_LENGTH, true);
    } else if (ppmState == PPM_STATE_FILL) {
        ppmOutput = LOW;
        currentChannelValue = getRcChannel_wrapper(ppmChannel);
        
        ppmChannel++;
        ppmState = PPM_STATE_PULSE;

        if (ppmChannel > PPM_CHANNELS) {
            ppmChannel = 0;
            timerAlarmWrite(timer, PPM_FRAME_LENGTH - usedFrameLength, true);
            usedFrameLength = 0;
        } else {
            usedFrameLength += currentChannelValue - PPM_PULSE_LENGTH;
            timerAlarmWrite(timer, currentChannelValue - PPM_PULSE_LENGTH, true);
        }
    }
    portEXIT_CRITICAL(&timerMux);
    digitalWrite(SERIAL1_TX, ppmOutput);
}

#endif

TaskHandle_t i2cResourceTask;
TaskHandle_t ioTask;
TaskHandle_t oledTask;

SSD1306Wire display(0x3c, &I2C1);
OledDisplay oledDisplay(&display);

QmuTactile buttonTrigger(PIN_BUTTON_TRIGGER);
QmuTactile buttonThumb(PIN_BUTTON_THUMB);
QmuTactile buttonJoystick(PIN_THUMB_JOYSTICK_SW);

void sensorCalibrate(struct gyroCalibration_t *cal, float sampleX, float sampleY, float sampleZ, const float dev)
{
    if (cal->state == CALIBARTION_NOT_DONE)
    {
        cal->state = CALIBRATION_IN_PROGRESS;
        cal->sampleCount = 0;
        devClear(&cal->deviation[AXIS_X]);
        devClear(&cal->deviation[AXIS_Y]);
        devClear(&cal->deviation[AXIS_Z]);
        cal->accumulatedValue[AXIS_X] = 0;
        cal->accumulatedValue[AXIS_Y] = 0;
        cal->accumulatedValue[AXIS_Z] = 0;
    }
    if (cal->state == CALIBRATION_IN_PROGRESS)
    {
        cal->sampleCount++;
        devPush(&cal->deviation[AXIS_X], sampleX);
        devPush(&cal->deviation[AXIS_Y], sampleY);
        devPush(&cal->deviation[AXIS_Z], sampleZ);
        cal->accumulatedValue[AXIS_X] += sampleX;
        cal->accumulatedValue[AXIS_Y] += sampleY;
        cal->accumulatedValue[AXIS_Z] += sampleZ;

        if (cal->sampleCount == 40)
        {

            if (
                devStandardDeviation(&cal->deviation[AXIS_X]) > dev ||
                devStandardDeviation(&cal->deviation[AXIS_Y]) > dev ||
                devStandardDeviation(&cal->deviation[AXIS_Z]) > dev)
            {
                cal->sampleCount = 0;
                devClear(&cal->deviation[AXIS_X]);
                devClear(&cal->deviation[AXIS_Y]);
                devClear(&cal->deviation[AXIS_Z]);
                cal->accumulatedValue[AXIS_X] = 0;
                cal->accumulatedValue[AXIS_Y] = 0;
                cal->accumulatedValue[AXIS_Z] = 0;
            }
            else
            {
                cal->zero[AXIS_X] = cal->accumulatedValue[AXIS_X] / cal->sampleCount;
                cal->zero[AXIS_Y] = cal->accumulatedValue[AXIS_Y] / cal->sampleCount;
                cal->zero[AXIS_Z] = cal->accumulatedValue[AXIS_Z] / cal->sampleCount;
                cal->state = CALIBRATION_DONE;
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);

#ifdef TRAINER_MODE_PPM
    pinMode(SERIAL1_TX, OUTPUT);
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onPpmTimer, true);
    timerAlarmWrite(timer, 12000, true);
    timerAlarmEnable(timer);
#endif

#ifdef TRAINER_MODE_SBUS
    sbusSerial.begin(100000, SERIAL_8E2, SERIAL1_RX, SERIAL1_TX, false, 100UL);
#endif

    I2C1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 50000);
    I2C2.begin(I2C2_SDA_PIN, I2C2_SCL_PIN, 100000);

    if (!mpu.begin(0x68, &I2C2, 0))
    {
        Serial.println("MPU6050 init fail");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 init success");

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    delay(50);

    oledDisplay.init();
    oledDisplay.setPage(OLED_PAGE_BEACON_STATUS);

    delay(50);

    buttonTrigger.start();
    buttonThumb.start();
    buttonJoystick.start();

    xTaskCreatePinnedToCore(
        i2cResourceTaskHandler, /* Function to implement the task */
        "imuTask",              /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        0,                      /* Priority of the task */
        &i2cResourceTask,       /* Task handle. */
        0);

    xTaskCreatePinnedToCore(
        ioTaskHandler, /* Function to implement the task */
        "outputTask",  /* Name of the task */
        10000,         /* Stack size in words */
        NULL,          /* Task input parameter */
        0,             /* Priority of the task */
        &ioTask,       /* Task handle. */
        0);

    xTaskCreatePinnedToCore(
        oledTaskHandler, /* Function to implement the task */
        "oledTask",  /* Name of the task */
        10000,         /* Stack size in words */
        NULL,          /* Task input parameter */
        0,             /* Priority of the task */
        &oledTask,       /* Task handle. */
        0);
}

//I do not get function pointers to object methods, no way...
int getRcChannel_wrapper(uint8_t channel)
{
    if (channel >= 0 && channel < SBUS_CHANNEL_COUNT)
    {
        return output.channels[channel];
    }
    else
    {
        return DEFAULT_CHANNEL_VALUE;
    }
}

void processJoystickAxis(uint8_t axis, uint8_t pin)
{
    thumbJoystick.raw[axis] = analogRead(pin);
    thumbJoystick.zeroed[axis] = thumbJoystick.calibration.zero[axis] - thumbJoystick.raw[axis];

    if (thumbJoystick.calibration.state == CALIBRATION_DONE)
    {

        if (thumbJoystick.zeroed[axis] > thumbJoystick.max[axis])
        {
            thumbJoystick.max[axis] = thumbJoystick.zeroed[axis];
        }

        if (thumbJoystick.zeroed[axis] < thumbJoystick.min[axis])
        {
            thumbJoystick.min[axis] = thumbJoystick.zeroed[axis];
        }

        if (thumbJoystick.zeroed[axis] > 0)
        {
            thumbJoystick.position[axis] = fscalef(thumbJoystick.zeroed[axis], 0, thumbJoystick.max[axis], 0.0f, 1.0f);
        }
        else
        {
            thumbJoystick.position[axis] = fscalef(thumbJoystick.zeroed[axis], thumbJoystick.min[axis], 0, -1.0f, 0.0f);
        }
    }
}

void outputSubtask()
{
    // Reset Z on each trigger press
    if (buttonTrigger.checkFlag(TACTILE_FLAG_EDGE_PRESSED))
    {
        imu.angle.z = 0;
    }

    // Z updates from gyro happens only when THUMB button is pressed
    if (!buttonThumb.checkFlag(TACTILE_FLAG_PRESSED)) {
        imu.angle.z = 0;
    }

    for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++)
    {
        output.channels[i] = DEFAULT_CHANNEL_VALUE;
    }

    if (buttonTrigger.getState() == TACTILE_STATE_LONG_PRESS) 
    {
        device.setActionEnabled(!device.getActionEnabled());
    }

    if (
        isnan(imu.angle.x) ||
        isnan(imu.angle.y) 
    ) {
        //TODO Data is broken, time to react or just do nothing
        device.setActionEnabled(false);
    }

    if (device.getActionEnabled()) 
    {
        output.channels[ROLL] = DEFAULT_CHANNEL_VALUE + angleToRcChannel(imu.angle.x);
        output.channels[PITCH] = DEFAULT_CHANNEL_VALUE + angleToRcChannel(imu.angle.y);
        output.channels[YAW] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(imu.angle.z) + joystickToRcChannel(thumbJoystick.position[AXIS_X]);
        output.channels[THROTTLE] = DEFAULT_CHANNEL_VALUE + joystickToRcChannel(thumbJoystick.position[AXIS_Y]);

        for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++) {
            output.channels[i] = constrain(output.channels[i], 1000, 2000);
        }

    }
}

void oledTaskHandler(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xPeriod = 200 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    /*
     * MPU6050 and OLED share the same I2C bus
     * To simplify the implementation and do not have to resolve resource conflicts,
     * both tasks are called in one thread pinned to the same core
     */
    for (;;)
    {
        /*
         * Process OLED display
         */
        oledDisplay.loop();

        // Put task to sleep
        vTaskDelayUntil(&xLastWakeTime, xPeriod); //There is a conflict on a I2C due to too much load. Have to put to sleep for a period of time instead
    }

    vTaskDelete(NULL);
}

void ioTaskHandler(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xPeriod = OUTPUT_UPDATE_TASK_MS / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        buttonTrigger.loop();
        buttonJoystick.loop();
        buttonThumb.loop();

        /*
         * Joystick handling
         */
        processJoystickAxis(AXIS_X, PIN_THUMB_JOYSTICK_X);
        processJoystickAxis(AXIS_Y, PIN_THUMB_JOYSTICK_Y);
        sensorCalibrate(&thumbJoystick.calibration, thumbJoystick.raw[AXIS_X], thumbJoystick.raw[AXIS_Y], 0, 3.0f);

        outputSubtask();

        // Put task to sleep
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }

    vTaskDelete(NULL);
}

void imuSubtask()
{
    static uint32_t prevMicros = 0;
    float dT = (micros() - prevMicros) * 0.000001f;
    prevMicros = micros();

    if (prevMicros > 0)
    {

        mpu.getEvent(&acc, &gyro, &temp);

        imu.accAngle.x = (atan(acc.acceleration.y / sqrt(pow(acc.acceleration.x, 2) + pow(acc.acceleration.z, 2))) * 180 / PI);
        imu.accAngle.y = (atan(-1 * acc.acceleration.x / sqrt(pow(acc.acceleration.y, 2) + pow(acc.acceleration.z, 2))) * 180 / PI);

        imu.gyro.x = (gyro.gyro.x * SENSORS_RADS_TO_DPS) - imu.gyroCalibration.zero[AXIS_X];
        imu.gyro.y = (gyro.gyro.y * SENSORS_RADS_TO_DPS) - imu.gyroCalibration.zero[AXIS_Y];
        imu.gyro.z = (gyro.gyro.z * SENSORS_RADS_TO_DPS) - imu.gyroCalibration.zero[AXIS_Z];

        imu.gyroNormalized.x = imu.gyro.x * dT;
        imu.gyroNormalized.y = imu.gyro.y * dT;
        imu.gyroNormalized.z = imu.gyro.z * dT;

        imu.angle.x = (0.95 * (imu.angle.x + imu.gyroNormalized.x)) + (0.05 * imu.accAngle.x);
        imu.angle.y = (0.95 * (imu.angle.y + imu.gyroNormalized.y)) + (0.05 * imu.accAngle.y);
        imu.angle.z = imu.angle.z + imu.gyroNormalized.z;

        /*
         * Calibration Routine
         */
        sensorCalibrate(&imu.gyroCalibration, imu.gyro.x, imu.gyro.y, imu.gyro.z, 3.0f);
    }
}

void i2cResourceTaskHandler(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xPeriod = MPU6050_UPDATE_TASK_MS / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    /*
     * MPU6050 and OLED share the same I2C bus
     * To simplify the implementation and do not have to resolve resource conflicts,
     * both tasks are called in one thread pinned to the same core
     */
    for (;;)
    {
        /*
        * Read gyro
        */
        imuSubtask();

        /*
         * Process OLED display
         */
        // oledDisplay.loop();

        // Put task to sleep
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }

    vTaskDelete(NULL);
}

int angleToRcChannel(float angle)
{
    const float value = fconstrainf(applyDeadband(angle, 5.0f), -45.0f, 45.0f); //5 deg deadband
    return (int)fscalef(value, -45.0f, 45.0f, -500, 500);
}

int joystickToRcChannel(float angle)
{
    const float value = fconstrainf(applyDeadband(angle, 0.02f), -1.0f, 1.0f);
    return (int)fscalef(value, -1.0f, 1.0f, -200, 200);
}

void loop()
{
    /* 
     * Send Trainer data in SBUS stream
     */
#ifdef TRAINER_MODE_SBUS
    if (millis() > nextSbusTaskMs)
    {
        sbusPreparePacket(sbusPacket, false, false, getRcChannel_wrapper);
        sbusSerial.write(sbusPacket, SBUS_PACKET_LENGTH);

        nextSbusTaskMs = millis() + SBUS_UPDATE_TASK_MS;
    }
#endif

    if (millis() > nextSerialTaskMs)
    {
        // Serial.println(String(imu.angle.z, 2) + " " + String(imu.gyro.z, 2));
        // Serial.println(String(imu.angle.x, 1) + " " + String(imu.angle.y, 1) + " " + String(imu.gyro.z, 1));
        // Serial.println("Zero: " + String(imu.gyroCalibration.zero[AXIS_X], 2) + " " + String(imu.gyroCalibration.zero[AXIS_Y], 2) + " " + String(imu.gyroCalibration.zero[AXIS_Z], 2));
        // Serial.println("Gyro: " + String(imu.gyro.x, 2) + " " + String(imu.gyro.y, 2) + " " + String(imu.gyro.z, 2));
        // Serial.println(String(devStandardDeviation(&imu.gyroCalDevX), 1) + " " + String(devStandardDeviation(&imu.gyroCalDevY), 1) + " " + String(devStandardDeviation(&imu.gyroCalDevZ), 1));
        // Serial.println(String(output.channels[ROLL]) + " " + String(output.channels[PITCH]) + " " + String(output.channels[THROTTLE]) + " " + String(output.channels[YAW]));
        // Serial.println("Zero: " + String(thumbJoystick.calibration.zero[AXIS_X], 2) + " " + String(thumbJoystick.calibration.zero[AXIS_Y], 2));
        // Serial.println(String(thumbJoystick.raw[AXIS_X]) + " " + String(thumbJoystick.raw[AXIS_Y]) + " " + digitalRead(PIN_THUMB_JOYSTICK_SW));
        // Serial.println(String(thumbJoystick.zeroed[AXIS_X]) + " " + String(thumbJoystick.max[AXIS_X]) + " " + String(thumbJoystick.min[AXIS_X]));
        // Serial.println(String(thumbJoystick.position[AXIS_X], 2) + " " + String(thumbJoystick.position[AXIS_Y], 2));

        nextSerialTaskMs = millis() + SERIAL_TASK_MS;
    }
}