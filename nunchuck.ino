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

typedef struct
{
    axis_t gyro; // in DPS
    axis_t accAngle;
    axis_t gyroNormalized;
    axis_t angle;
    axis_t gyroZero;

    stdev_t gyroCalDevX;
    stdev_t gyroCalDevY;
    stdev_t gyroCalDevZ;
    uint32_t gyroCalSampleCount;
    axis_t accumulatedValue;
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

enum calibrationState_e {
    CALIBARTION_NOT_DONE,
    CALIBRATION_IN_PROGRESS,
    CALIBRATION_DONE
};

uint8_t gyroCalibrationState = CALIBARTION_NOT_DONE;

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

                imu.gyro.x = (gyro.gyro.x * SENSORS_RADS_TO_DPS) - imu.gyroZero.x;
                imu.gyro.y = (gyro.gyro.y * SENSORS_RADS_TO_DPS) - imu.gyroZero.y;
                imu.gyro.z = (gyro.gyro.z * SENSORS_RADS_TO_DPS) - imu.gyroZero.z;

                imu.gyroNormalized.x = imu.gyro.x * dT;
                imu.gyroNormalized.y = imu.gyro.y * dT;
                imu.gyroNormalized.z = imu.gyro.z * dT;

                imu.angle.x = (0.95 * (imu.angle.x + imu.gyroNormalized.x)) + (0.05 * imu.accAngle.x);
                imu.angle.y = (0.95 * (imu.angle.y + imu.gyroNormalized.y)) + (0.05 * imu.accAngle.y);

                /*
                 * Calibration Routine
                 */
                if (gyroCalibrationState == CALIBARTION_NOT_DONE) {
                    gyroCalibrationState = CALIBRATION_IN_PROGRESS;
                    imu.gyroCalSampleCount = 0;
                    devClear(&imu.gyroCalDevX);
                    devClear(&imu.gyroCalDevY);
                    devClear(&imu.gyroCalDevZ);
                    imu.accumulatedValue.x = 0;
                    imu.accumulatedValue.y = 0;
                    imu.accumulatedValue.z = 0;
                }
                if (gyroCalibrationState == CALIBRATION_IN_PROGRESS) {
                    imu.gyroCalSampleCount++;
                    devPush(&imu.gyroCalDevX, imu.gyro.x);
                    devPush(&imu.gyroCalDevY, imu.gyro.y);
                    devPush(&imu.gyroCalDevZ, imu.gyro.z);
                    imu.accumulatedValue.x += imu.gyro.x;
                    imu.accumulatedValue.y += imu.gyro.y;
                    imu.accumulatedValue.z += imu.gyro.z;

                    if (imu.gyroCalSampleCount == 40) {
                        
                        if (
                            devStandardDeviation(&imu.gyroCalDevX) > 3.0f ||
                            devStandardDeviation(&imu.gyroCalDevY) > 3.0f ||
                            devStandardDeviation(&imu.gyroCalDevZ) > 3.0f
                        ) {
                            imu.gyroCalSampleCount = 0;
                            devClear(&imu.gyroCalDevX);
                            devClear(&imu.gyroCalDevY);
                            devClear(&imu.gyroCalDevZ);
                            imu.accumulatedValue.x = 0;
                            imu.accumulatedValue.y = 0;
                            imu.accumulatedValue.z = 0;
                        } else {
                            imu.gyroZero.x = imu.accumulatedValue.x / imu.gyroCalSampleCount;
                            imu.gyroZero.y = imu.accumulatedValue.y / imu.gyroCalSampleCount;
                            imu.gyroZero.z = imu.accumulatedValue.z / imu.gyroCalSampleCount;
                            gyroCalibrationState = CALIBRATION_DONE;
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
    // currentMs = millis();

    /*
     * Read MPU6050 and compute current angle
     */

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
        // Serial.println("Zero: " + String(imu.gyroZero.x, 2) + " " + String(imu.gyroZero.y, 2) + " " + String(imu.gyroZero.z, 2));
        // Serial.println("Gyro: " + String(imu.gyro.x, 2) + " " + String(imu.gyro.y, 2) + " " + String(imu.gyro.z, 2));
        // Serial.println(String(devStandardDeviation(&imu.gyroCalDevX), 1) + " " + String(devStandardDeviation(&imu.gyroCalDevY), 1) + " " + String(devStandardDeviation(&imu.gyroCalDevZ), 1));

        nextSerialTaskMs = millis() + SERIAL_TASK_MS;
    }
}
