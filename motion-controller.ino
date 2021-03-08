#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <QmuTactile.h>
#include "sbus.h"
#include "math.h"
#include "types.h"
#include "SSD1306.h"
#include "oled_display.h"
#include "device_node.h"

#define SBUS_UPDATE_TASK_MS 15
#define MPU6050_UPDATE_TASK_MS 30
#define OUTPUT_UPDATE_TASK_MS 20
#define SERIAL_TASK_MS 50
#define SERIAL1_RX 25
#define SERIAL1_TX 14
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

#define PIN_BUTTON_UP 4
#define PIN_BUTTON_DOWN 2
#define PIN_BUTTON_TRIGGER 15

#define PIN_THUMB_JOYSTICK_X 13
#define PIN_THUMB_JOYSTICK_Y 33
#define PIN_THUMB_JOYSTICK_SW 32

Adafruit_MPU6050 mpu;
sensors_event_t acc, gyro, temp;

uint32_t nextSbusTaskMs = 0;
uint32_t nextSerialTaskMs = 0;

imuData_t imu;
dataOutput_t output;
thumb_joystick_t thumbJoystick;
DeviceNode device;

uint8_t sbusPacket[SBUS_PACKET_LENGTH] = {0};
HardwareSerial sbusSerial(1);
TaskHandle_t i2cResourceTask;
TaskHandle_t outputTask;

SSD1306 display(0x3c, I2C_SDA_PIN, I2C_SCL_PIN);
OledDisplay oledDisplay(&display);

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

    sbusSerial.begin(100000, SERIAL_8E2, SERIAL1_RX, SERIAL1_TX, false, 100UL);

    if (!mpu.begin())
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

    pinMode(PIN_THUMB_JOYSTICK_SW, INPUT_PULLUP);

    xTaskCreatePinnedToCore(
        i2cResourceTaskHandler, /* Function to implement the task */
        "imuTask",              /* Name of the task */
        10000,                  /* Stack size in words */
        NULL,                   /* Task input parameter */
        0,                      /* Priority of the task */
        &i2cResourceTask,       /* Task handle. */
        0);

    xTaskCreatePinnedToCore(
        outputTaskHandler, /* Function to implement the task */
        "outputTask",      /* Name of the task */
        10000,             /* Stack size in words */
        NULL,              /* Task input parameter */
        0,                 /* Priority of the task */
        &outputTask,       /* Task handle. */
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

void outputTaskHandler(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xPeriod = OUTPUT_UPDATE_TASK_MS / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    uint8_t prevTriggerButtonState;
    uint8_t triggerButtonState;

    for (;;)
    {

        /*
         * Joystick handling
         */
        processJoystickAxis(AXIS_X, PIN_THUMB_JOYSTICK_X);
        processJoystickAxis(AXIS_Y, PIN_THUMB_JOYSTICK_Y);
        sensorCalibrate(&thumbJoystick.calibration, thumbJoystick.raw[AXIS_X], thumbJoystick.raw[AXIS_Y], 0, 3.0f);

        triggerButtonState = digitalRead(PIN_BUTTON_TRIGGER);

        //On trigger press, reset yaw
        if (triggerButtonState == LOW && prevTriggerButtonState == HIGH)
        {
            imu.angle.z = 0;
        }

        for (uint8_t i = 0; i < SBUS_CHANNEL_COUNT; i++)
        {
            output.channels[i] = DEFAULT_CHANNEL_VALUE;
        }

        if (
            isnan(imu.angle.x) ||
            isnan(imu.angle.y) ||
            digitalRead(PIN_BUTTON_TRIGGER) != LOW)
        {
            //TODO Data is broken, time to react or just do nothing
            device.setActionEnabled(false);
        }
        else
        {
            device.setActionEnabled(true);
            output.channels[ROLL] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(imu.angle.x);
            output.channels[PITCH] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(imu.angle.y);
            output.channels[YAW] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(imu.angle.z) + joystickToRcChannel(thumbJoystick.position[AXIS_X]);
            output.channels[THROTTLE] = DEFAULT_CHANNEL_VALUE + joystickToRcChannel(thumbJoystick.position[AXIS_Y]);

            if (digitalRead(PIN_BUTTON_UP) == LOW)
            {
                output.channels[THROTTLE] += THROTTLE_BUTTON_STEP;
            }
            if (digitalRead(PIN_BUTTON_DOWN) == LOW)
            {
                output.channels[THROTTLE] -= THROTTLE_BUTTON_STEP;
            }
        }

        prevTriggerButtonState = triggerButtonState;

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
        oledDisplay.loop();

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
    const float value = fconstrainf(applyDeadband(angle, 0.05f), -1.0f, 1.0f);
    return (int)fscalef(value, -1.0f, 1.0f, -200, 200);
}

void loop()
{
    /* 
     * Send Trainer data in SBUS stream
     */
    if (millis() > nextSbusTaskMs)
    {
        sbusPreparePacket(sbusPacket, false, false, getRcChannel_wrapper);
        sbusSerial.write(sbusPacket, SBUS_PACKET_LENGTH);

        nextSbusTaskMs = millis() + SBUS_UPDATE_TASK_MS;
    }

    if (millis() > nextSerialTaskMs)
    {
        // Serial.println(String(imu.angle.z, 2) + " " + String(imu.gyro.z, 2));
        // Serial.println(String(imu.angle.x, 1) + " " + String(imu.angle.y, 1) + " " + String(imu.gyro.z, 1));
        // Serial.println("Zero: " + String(imu.gyroCalibration.zero[AXIS_X], 2) + " " + String(imu.gyroCalibration.zero[AXIS_Y], 2) + " " + String(imu.gyroCalibration.zero[AXIS_Z], 2));
        // Serial.println("Gyro: " + String(imu.gyro.x, 2) + " " + String(imu.gyro.y, 2) + " " + String(imu.gyro.z, 2));
        // Serial.println(String(devStandardDeviation(&imu.gyroCalDevX), 1) + " " + String(devStandardDeviation(&imu.gyroCalDevY), 1) + " " + String(devStandardDeviation(&imu.gyroCalDevZ), 1));
        Serial.println(String(output.channels[ROLL]) + " " + String(output.channels[PITCH]) + " " + String(output.channels[THROTTLE]) + " " + String(output.channels[YAW]));
        // Serial.println("Zero: " + String(thumbJoystick.calibration.zero[AXIS_X], 2) + " " + String(thumbJoystick.calibration.zero[AXIS_Y], 2));
        // Serial.println(String(thumbJoystick.raw[AXIS_X]) + " " + String(thumbJoystick.raw[AXIS_Y]) + " " + digitalRead(PIN_THUMB_JOYSTICK_SW));
        // Serial.println(String(thumbJoystick.zeroed[AXIS_X]) + " " + String(thumbJoystick.max[AXIS_X]) + " " + String(thumbJoystick.min[AXIS_X]));
        // Serial.println(String(thumbJoystick.position[AXIS_X], 2) + " " + String(thumbJoystick.position[AXIS_Y], 2));

        nextSerialTaskMs = millis() + SERIAL_TASK_MS;
    }
}
