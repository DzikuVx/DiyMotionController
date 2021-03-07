#pragma once

#ifndef TYPES_H
#define TYPES_H

#define AXIS_COUNT 3
#define SBUS_CHANNEL_COUNT 16
#define DEFAULT_CHANNEL_VALUE 1500
#define THROTTLE_BUTTON_STEP 100

#include <Arduino.h>

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

enum calibrationState_e
{
    CALIBARTION_NOT_DONE,
    CALIBRATION_IN_PROGRESS,
    CALIBRATION_DONE
};

struct gyroCalibration_t
{
    stdev_t deviation[AXIS_COUNT];
    float accumulatedValue[AXIS_COUNT];
    float zero[AXIS_COUNT];
    uint32_t sampleCount;
    uint8_t state = CALIBARTION_NOT_DONE;
};

typedef struct
{
    axis_t gyro;           // in DPS
    axis_t accAngle;       // in degrees
    axis_t gyroNormalized; // in dps * dT
    axis_t angle;          // angle from complimentary filter

    gyroCalibration_t gyroCalibration;

} imuData_t;

typedef struct
{
    int raw[2];
    int min[2];
    int max[2];
    int zeroed[2];
    float position[2]; // this one is -1.0f to 1.0f
    gyroCalibration_t calibration;
} thumb_joystick_t;

typedef struct
{
    uint16_t channels[SBUS_CHANNEL_COUNT];

} dataOutput_t;


#endif