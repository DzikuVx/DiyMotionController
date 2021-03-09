#pragma once

#ifndef QMU_TACTILE_H
#define QMU_TACTILE_H

#include "Arduino.h"

enum tactileStateFlags
{
  TACTILE_STATE_NONE,
  TACTILE_STATE_SHORT_PRESS,
  TACTILE_STATE_LONG_PRESS,
  TACTILE_STATE_PRESSING
};

enum tactileFlagsFlags_e
{
  TACTILE_FLAG_NONE = 0,                  //  0
  TACTILE_FLAG_PRESSED = 1 << 0,          //  1
  TACTILE_FLAG_EDGE_PRESSED = 1 << 1,     //  2
  TACTILE_FLAG_EDGE_NOT_PRESSED = 1 << 2, // 4 
};

#define TACTILE_MIN_PRESS_TIME 50
#define TACTILE_LONG_PRESS_TIME 1000
#define TACTILE_PRESSING_TIME 400

class QmuTactile
{
public:
  QmuTactile(uint8_t pin);
  void loop(void);
  void start(void);
  tactileStateFlags getState(void);
  uint8_t getFlags(void);
  bool checkFlag(tactileFlagsFlags_e flag);

private:
  uint8_t _pin;
  uint8_t _previousPinState = HIGH;
  uint32_t _pressMillis = 0;
  tactileStateFlags _state = TACTILE_STATE_NONE;
  uint32_t _nextPressingEvent = 0;
  uint8_t _flags;
};

#endif