#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include "sbus.h"
#include "math.h"

#define SBUS_UPDATE_TASK_MS 15
#define MPU6050_UPDATE_TASK_MS 25
#define SERIAL_TASK_MS 50
#define SERIAL1_RX 25
#define SERIAL1_TX 14

Adafruit_MPU6050 mpu;
sensors_event_t acc, gyro, temp;

uint32_t nextSbusTaskMs = 0;
uint32_t nextAccTaskMs = 0;
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

typedef enum {
    AXIS_X      = 0,
    AXIS_Y      = 1,
    AXIS_Z      = 2
} axis_definition_e;

#define AXIS_COUNT 3

enum calibrationState_e {
    CALIBARTION_NOT_DONE,
    CALIBRATION_IN_PROGRESS,
    CALIBRATION_DONE
};

typedef struct {
    stdev_t deviation[AXIS_COUNT];
    float accumulatedValue[AXIS_COUNT];
    float zero[AXIS_COUNT];
    uint32_t sampleCount;
    uint8_t state = CALIBARTION_NOT_DONE;
} gyroCalibration_t;

typedef struct
{
    axis_t gyro; // in DPS
    axis_t accAngle;        // in degrees
    axis_t gyroNormalized;  // in dps * dT
    axis_t angle;           // angle from complimentary filter

    gyroCalibration_t gyroCalibration;

} imuData_t;

imuData_t imu;

uint8_t sbusPacket[SBUS_PACKET_LENGTH] = {0};
HardwareSerial sbusSerial(1);
TaskHandle_t imuTask;

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

    xTaskCreatePinnedToCore(
        imuTaskHandler, /* Function to implement the task */
        "imuTask",      /* Name of the task */
        10000,          /* Stack size in words */
        NULL,           /* Task input parameter */
        0,              /* Priority of the task */
        &imuTask,       /* Task handle. */
        0);
}

//I do not get function pointers to object methods, no way...
int getRcChannel_wrapper(uint8_t channel)
{
    if (channel == 0)
    {
        return 1700;
    }
    else if (channel == 1)
    {
        return 1400;
    }
    else if (channel == 2)
    {
        return 1800;
    }
    else if (channel == 3)
    {
        return 1200;
    }
    else
    {
        return 1500;
    }
}

void imuTaskHandler(void *pvParameters)
{

    for (;;)
    {

        if (millis() > nextAccTaskMs)
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

                /*
                 * Calibration Routine
                 */
                if (imu.gyroCalibration.state == CALIBARTION_NOT_DONE) {
                    imu.gyroCalibration.state = CALIBRATION_IN_PROGRESS;
                    imu.gyroCalibration.sampleCount = 0;
                    devClear(&imu.gyroCalibration.deviation[AXIS_X]);
                    devClear(&imu.gyroCalibration.deviation[AXIS_Y]);
                    devClear(&imu.gyroCalibration.deviation[AXIS_Z]);
                    imu.gyroCalibration.accumulatedValue[AXIS_X] = 0;
                    imu.gyroCalibration.accumulatedValue[AXIS_Y] = 0;
                    imu.gyroCalibration.accumulatedValue[AXIS_Z] = 0;
                }
                if (imu.gyroCalibration.state == CALIBRATION_IN_PROGRESS) {
                    imu.gyroCalibration.sampleCount++;
                    devPush(&imu.gyroCalibration.deviation[AXIS_X], imu.gyro.x);
                    devPush(&imu.gyroCalibration.deviation[AXIS_Y], imu.gyro.y);
                    devPush(&imu.gyroCalibration.deviation[AXIS_Z], imu.gyro.z);
                    imu.gyroCalibration.accumulatedValue[AXIS_X] += imu.gyro.x;
                    imu.gyroCalibration.accumulatedValue[AXIS_Y] += imu.gyro.y;
                    imu.gyroCalibration.accumulatedValue[AXIS_Z] += imu.gyro.z;

                    if (imu.gyroCalibration.sampleCount == 40) {
                        
                        if (
                            devStandardDeviation(&imu.gyroCalibration.deviation[AXIS_X]) > 3.0f ||
                            devStandardDeviation(&imu.gyroCalibration.deviation[AXIS_Y]) > 3.0f ||
                            devStandardDeviation(&imu.gyroCalibration.deviation[AXIS_Z]) > 3.0f
                        ) {
                            imu.gyroCalibration.sampleCount = 0;
                            devClear(&imu.gyroCalibration.deviation[AXIS_X]);
                            devClear(&imu.gyroCalibration.deviation[AXIS_Y]);
                            devClear(&imu.gyroCalibration.deviation[AXIS_Z]);
                            imu.gyroCalibration.accumulatedValue[AXIS_X] = 0;
                            imu.gyroCalibration.accumulatedValue[AXIS_Y] = 0;
                            imu.gyroCalibration.accumulatedValue[AXIS_Z] = 0;
                        } else {
                            imu.gyroCalibration.zero[AXIS_X] = imu.gyroCalibration.accumulatedValue[AXIS_X] / imu.gyroCalibration.sampleCount;
                            imu.gyroCalibration.zero[AXIS_Y] = imu.gyroCalibration.accumulatedValue[AXIS_Y] / imu.gyroCalibration.sampleCount;
                            imu.gyroCalibration.zero[AXIS_Z] = imu.gyroCalibration.accumulatedValue[AXIS_Z] / imu.gyroCalibration.sampleCount;
                            imu.gyroCalibration.state = CALIBRATION_DONE;
                        }

                    }
                }

            }

            nextAccTaskMs = millis() + MPU6050_UPDATE_TASK_MS;
        }
    }

    vTaskDelete(NULL);
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
        // Serial.println(String(imu.angle.x, 1) + " " + String(imu.angle.y, 1) + " " + String(imu.gyro.z, 1));
        Serial.println("Zero: " + String(imu.gyroCalibration.zero[AXIS_X], 2) + " " + String(imu.gyroCalibration.zero[AXIS_Y], 2) + " " + String(imu.gyroCalibration.zero[AXIS_Z], 2));
        Serial.println("Gyro: " + String(imu.gyro.x, 2) + " " + String(imu.gyro.y, 2) + " " + String(imu.gyro.z, 2));
        // Serial.println(String(devStandardDeviation(&imu.gyroCalDevX), 1) + " " + String(devStandardDeviation(&imu.gyroCalDevY), 1) + " " + String(devStandardDeviation(&imu.gyroCalDevZ), 1));

        nextSerialTaskMs = millis() + SERIAL_TASK_MS;
    }
}
