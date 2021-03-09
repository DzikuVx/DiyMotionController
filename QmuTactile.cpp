#include "Arduino.h"
#include "QmuTactile.h"

QmuTactile::QmuTactile(uint8_t pin) {
    _pin = pin;
}

void QmuTactile::loop(void) {

    uint8_t pinState = digitalRead(_pin);
    _state = TACTILE_STATE_NONE;
    _flags = TACTILE_FLAG_NONE;

    //Press moment
    if (pinState == LOW && _previousPinState == HIGH) {
        _pressMillis = millis();
        _nextPressingEvent = 0;
    }

    const uint32_t buttonTime = abs(millis() - _pressMillis);

    //Pressing
    if (pinState == LOW && buttonTime > TACTILE_PRESSING_TIME) {
        if (millis() > _nextPressingEvent) {
            _state = TACTILE_STATE_PRESSING;
            _nextPressingEvent = millis() + 500;
        }
    }

    //Release moment
    if (pinState == HIGH && _previousPinState == LOW) {

        if (buttonTime > TACTILE_LONG_PRESS_TIME) {
            _state = TACTILE_STATE_LONG_PRESS;
        } else if (buttonTime > TACTILE_MIN_PRESS_TIME) {
            _state = TACTILE_STATE_SHORT_PRESS;
        }

    }

    // _previousPosition = _position;
    if (pinState == LOW) {
        _flags |= TACTILE_FLAG_PRESSED;
    }
    if (pinState == LOW && _previousPinState == HIGH) {
        _flags |= TACTILE_FLAG_EDGE_PRESSED;
    }
    if (pinState == HIGH && _previousPinState == LOW) {
        _flags |= TACTILE_FLAG_EDGE_NOT_PRESSED;
    }

    _previousPinState = pinState;
}

void QmuTactile::start(void) {
    pinMode(_pin, INPUT_PULLUP);
}

tactileStateFlags QmuTactile::getState(void) {
    return _state;
}

uint8_t QmuTactile::getFlags(void) {
    return _flags;
}

bool QmuTactile::checkFlag(tactileFlagsFlags_e flag) {
    return (_flags & flag);
}