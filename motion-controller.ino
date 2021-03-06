#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <QmuTactile.h>
#include "sbus.h"
#include "math.h"

#define SBUS_UPDATE_TASK_MS 15
#define MPU6050_UPDATE_TASK_MS 30
#define OUTPUT_UPDATE_TASK_MS 20
#define SERIAL_TASK_MS 50
#define SERIAL1_RX 25
#define SERIAL1_TX 14

#define PIN_BUTTON_UP 4
#define PIN_BUTTON_DOWN 2
#define PIN_BUTTON_TRIGGER 15

Adafruit_MPU6050 mpu;
sensors_event_t acc, gyro, temp;

uint32_t nextSbusTaskMs = 0;
uint32_t nextSerialTaskMs = 0;

float accAngleX;
float accAngleY;

float gyroX;
float gyroY;
float gyroZ;

typedef struct
{
    float x, y, z;
} axis_t;

typedef enum
{
    AXIS_X = 0,
    AXIS_Y = 1,
    AXIS_Z = 2
} axis_definition_e;

typedef enum
{
    ROLL = 0,
    PITCH = 1,
    THROTTLE = 2,
    YAW = 3
} channel_functions_e;

#define AXIS_COUNT 3
#define SBUS_CHANNEL_COUNT 16
#define DEFAULT_CHANNEL_VALUE 1500
#define THROTTLE_BUTTON_STEP 100

enum calibrationState_e
{
    CALIBARTION_NOT_DONE,
    CALIBRATION_IN_PROGRESS,
    CALIBRATION_DONE
};

typedef struct
{
    stdev_t deviation[AXIS_COUNT];
    float accumulatedValue[AXIS_COUNT];
    float zero[AXIS_COUNT];
    uint32_t sampleCount;
    uint8_t state = CALIBARTION_NOT_DONE;
} gyroCalibration_t;

typedef struct
{
    axis_t gyro;           // in DPS
    axis_t accAngle;       // in degrees
    axis_t gyroNormalized; // in dps * dT
    axis_t angle;          // angle from complimentary filter

    gyroCalibration_t gyroCalibration;

} imuData_t;

imuData_t imu;

typedef struct
{
    uint16_t channels[SBUS_CHANNEL_COUNT];

} dataOutput_t;

dataOutput_t output;

uint8_t sbusPacket[SBUS_PACKET_LENGTH] = {0};
HardwareSerial sbusSerial(1);
TaskHandle_t imuTask;
TaskHandle_t outputTask;

QmuTactile buttonUp(PIN_BUTTON_UP);
QmuTactile buttonDown(PIN_BUTTON_DOWN);
QmuTactile buttonTrigger(PIN_BUTTON_TRIGGER);

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

    buttonUp.start();
    buttonDown.start();
    buttonTrigger.start();

    xTaskCreatePinnedToCore(
        imuTaskHandler, /* Function to implement the task */
        "imuTask",      /* Name of the task */
        10000,          /* Stack size in words */
        NULL,           /* Task input parameter */
        0,              /* Priority of the task */
        &imuTask,       /* Task handle. */
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

void outputTaskHandler(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xPeriod = OUTPUT_UPDATE_TASK_MS / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
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
        }
        else
        {
            output.channels[ROLL] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(imu.angle.x);
            output.channels[PITCH] = DEFAULT_CHANNEL_VALUE - angleToRcChannel(imu.angle.y);

            if (digitalRead(PIN_BUTTON_UP) == LOW)
            {
                output.channels[THROTTLE] += THROTTLE_BUTTON_STEP;
            }
            if (digitalRead(PIN_BUTTON_DOWN) == LOW)
            {
                output.channels[THROTTLE] -= THROTTLE_BUTTON_STEP;
            }
        }

        // Put task to sleep
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }

    vTaskDelete(NULL);
}

void imuTaskHandler(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xPeriod = MPU6050_UPDATE_TASK_MS / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /*
        * Read gyro
        */
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

            /*
             * Calibration Routine
             */
            if (imu.gyroCalibration.state == CALIBARTION_NOT_DONE)
            {
                imu.gyroCalibration.state = CALIBRATION_IN_PROGRESS;
                imu.gyroCalibration.sampleCount = 0;
                devClear(&imu.gyroCalibration.deviation[AXIS_X]);
                devClear(&imu.gyroCalibration.deviation[AXIS_Y]);
                devClear(&imu.gyroCalibration.deviation[AXIS_Z]);
                imu.gyroCalibration.accumulatedValue[AXIS_X] = 0;
                imu.gyroCalibration.accumulatedValue[AXIS_Y] = 0;
                imu.gyroCalibration.accumulatedValue[AXIS_Z] = 0;
            }
            if (imu.gyroCalibration.state == CALIBRATION_IN_PROGRESS)
            {
                imu.gyroCalibration.sampleCount++;
                devPush(&imu.gyroCalibration.deviation[AXIS_X], imu.gyro.x);
                devPush(&imu.gyroCalibration.deviation[AXIS_Y], imu.gyro.y);
                devPush(&imu.gyroCalibration.deviation[AXIS_Z], imu.gyro.z);
                imu.gyroCalibration.accumulatedValue[AXIS_X] += imu.gyro.x;
                imu.gyroCalibration.accumulatedValue[AXIS_Y] += imu.gyro.y;
                imu.gyroCalibration.accumulatedValue[AXIS_Z] += imu.gyro.z;

                if (imu.gyroCalibration.sampleCount == 40)
                {

                    if (
                        devStandardDeviation(&imu.gyroCalibration.deviation[AXIS_X]) > 3.0f ||
                        devStandardDeviation(&imu.gyroCalibration.deviation[AXIS_Y]) > 3.0f ||
                        devStandardDeviation(&imu.gyroCalibration.deviation[AXIS_Z]) > 3.0f)
                    {
                        imu.gyroCalibration.sampleCount = 0;
                        devClear(&imu.gyroCalibration.deviation[AXIS_X]);
                        devClear(&imu.gyroCalibration.deviation[AXIS_Y]);
                        devClear(&imu.gyroCalibration.deviation[AXIS_Z]);
                        imu.gyroCalibration.accumulatedValue[AXIS_X] = 0;
                        imu.gyroCalibration.accumulatedValue[AXIS_Y] = 0;
                        imu.gyroCalibration.accumulatedValue[AXIS_Z] = 0;
                    }
                    else
                    {
                        imu.gyroCalibration.zero[AXIS_X] = imu.gyroCalibration.accumulatedValue[AXIS_X] / imu.gyroCalibration.sampleCount;
                        imu.gyroCalibration.zero[AXIS_Y] = imu.gyroCalibration.accumulatedValue[AXIS_Y] / imu.gyroCalibration.sampleCount;
                        imu.gyroCalibration.zero[AXIS_Z] = imu.gyroCalibration.accumulatedValue[AXIS_Z] / imu.gyroCalibration.sampleCount;
                        imu.gyroCalibration.state = CALIBRATION_DONE;
                    }
                }
            }
        }

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

void loop()
{
    buttonTrigger.loop();
    buttonUp.loop();
    buttonDown.loop();

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
        // Serial.println(String(imu.angle.x, 1) + " " + String(imu.angle.y, 1) + " " + String(imu.gyro.z, 1));
        // Serial.println("Zero: " + String(imu.gyroCalibration.zero[AXIS_X], 2) + " " + String(imu.gyroCalibration.zero[AXIS_Y], 2) + " " + String(imu.gyroCalibration.zero[AXIS_Z], 2));
        // Serial.println("Gyro: " + String(imu.gyro.x, 2) + " " + String(imu.gyro.y, 2) + " " + String(imu.gyro.z, 2));
        // Serial.println(String(devStandardDeviation(&imu.gyroCalDevX), 1) + " " + String(devStandardDeviation(&imu.gyroCalDevY), 1) + " " + String(devStandardDeviation(&imu.gyroCalDevZ), 1));
        Serial.println(String(output.channels[ROLL]) + " " + String(output.channels[PITCH]) + " " + String(output.channels[THROTTLE]) + " " + String(output.channels[YAW]));

        nextSerialTaskMs = millis() + SERIAL_TASK_MS;
    }
}
