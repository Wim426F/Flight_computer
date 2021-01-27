#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <flight_controls.h>

typedef enum {
  RECEIVE_SUCCES = 0,
  RECEIVE_FAILED = 1,
  TRANSMIT_SUCCES = 2,
  TRANSMIT_FAILED = 3
} rc_status_t;

class RadioCommunication
{
private:
public:
  int setChannel(int channel = 1);
  int setTxPower(int power = 8);
  int setBaudRate(int baud = 4800);
  rc_status_t sendBytes();
  rc_status_t parseIncomingBytes();
};

extern const float rc_txpower[];
extern const uint32_t rc_baudrate[];



#endif
