#ifndef HC12_H
#define HC12_H

#include <flight_controls.h>

class HC12
{
private:
public:
  typedef enum ACK
  {
    OK = 0x00,           // Operation succeeded. No Error.
    UNKOWN_ERROR = 0x01, // Device communication failed for unknown reason
    INVALID_CRC = 0x02,  // calculated a different CRC / data transmission from Master failed
    VERIFY_ERROR = 0x03, // Interface did a successful write operation, but the read back data did not match
    CMD_FAILED = 0x04,   // Device communication failed
    CMD_INVALID = 0x05,
    PARAM_INVALID = 0x06,
    NOT_READY = 0x07,
    TIMEOUT = 0x08
  } rc_ack_t;

  enum protocol
  {
    TRANSMITTER_CMD = 0x54, // 'T'
    RECEIVER_ANSWER = 0x48, // 'H'
  };

  enum MODE
  {
    NORMAL = 0x4e,      // 'N'
    SET_CHANNEL = 0x43, // 'C'
    SET_TXPOWER = 0x50, // 'P'
    SET_BAUD = 0x42,    // 'B'
    SET_MODE = 0x4d,    // 'M'
    SET_DEFAULT = 0x44  // 'D'
  };

  rc_ack_t setChannel(int channel = 1);
  rc_ack_t setTxPower(int power = 8);
  rc_ack_t setBaudRate(int baud = 7);
  rc_ack_t setTransmitMode(int new_mode);
  rc_ack_t setAllDefault();
  void getVersion();
  void sleep();
  void wakeUp();
  void sendData(enum HC12::ACK ack);
  rc_ack_t readData();
  bool isConnected = false;
  bool waitingForCommand = true;
};

extern uint8_t last_channel;
extern uint8_t last_baud;
extern uint8_t last_txpower;
extern uint8_t last_txmode;

extern const float rc_txpower[];
extern const uint32_t rc_baudrate[];

#endif
