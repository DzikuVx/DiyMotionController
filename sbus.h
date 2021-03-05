
#ifndef SBUS_INPUT
#define SBUS_INPUT

#include "Arduino.h"

#define SBUS_PACKET_LENGTH 25

enum sbusProtocolStates {
    SBUS_DECODING_STATE_IDLE,
    SBUS_DECODING_STATE_IN_PROGRESS
};

class TxInput
{
  public:
  	virtual ~TxInput() {}
    virtual void start(void) {};
    virtual void stop(void) {};
    virtual bool isReceiving(void) { return false; };
    virtual void loop(void) {};
};

class SbusInput : public TxInput
{
  public:
  	SbusInput(HardwareSerial &serial);
    void start(void);
    void restart(void);
    void loop(void);
    bool isReceiving(void);
    void recoverStuckFrames(void);
    void (* setRcChannelCallback)(uint8_t channel, int value, int offset);
  private:
  	HardwareSerial &_serial;
    uint32_t _frameDecodingStartedAt = 0;
    uint32_t _frameDecodingEndedAt = 0 ;
    uint8_t _protocolState = SBUS_DECODING_STATE_IDLE;
	  void sbusRead(void);
    void sbusToChannels(byte buffer[]);
};

void sbusPreparePacket(uint8_t packet[], bool isSignalLoss, bool isFailsafe, int (* rcChannelGetCallback)(uint8_t));

#endif

