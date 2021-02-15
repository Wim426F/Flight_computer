#include <string>
#include <communication.h>
#include <globals.h>

using namespace std;
String set_channel;
String set_baudrate;

const float rc_txpower[8] = {0.8, 1.6, 3.2, 6.3, 12, 25, 50, 100};
const uint32_t rc_baudrate[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};

uint8_t packet[16];
uint8_t send_packet[8];
uint8_t i;

uint16_t local_checksum = 0;
uint8_t pre_checksum = 0;

const uint8_t DATA_BYTES = 255;
const uint8_t CONFIG_BYTES = 127;

int RadioCommunication::setChannel(int channel)
{
  #ifdef TARGET_TEENSY35
  String set_channel;
  if (channel < 10)
  {
    set_channel = "AT+C00" + (String)channel;
  }
  else
  {
    set_channel = "AT+C0" + (String)channel;
  }

  digitalWrite(HC12_COMMAND_MODE, LOW);
  delay(40);
  HC12.println(set_channel);
  delay(20);
  digitalWrite(HC12_COMMAND_MODE, HIGH);
  delay(80);
  Serial.println("channel set");
  #endif

  return 0;
}
int RadioCommunication::setTxPower(int power)
{
  #ifdef TARGET_TEENSY35
  digitalWrite(HC12_COMMAND_MODE, LOW);
  delay(40);
  HC12.println((String) "AT+P" + power);
  delay(20);
  digitalWrite(HC12_COMMAND_MODE, HIGH);
  delay(80);
  Serial.println("power set");
  #endif
  return 0;
}
int RadioCommunication::setBaudRate(int baud)
{
  #ifdef TARGET_TEENSY35
  digitalWrite(HC12_COMMAND_MODE, LOW);
  delay(40);
  HC12.println("AT+B" + (String)baud);
  HC12.end();
  delay(20);
  digitalWrite(HC12_COMMAND_MODE, HIGH);
  delay(80);

  HC12.begin(baud);
  while (!HC12)
  delay(50);
  Serial.println("baudrate set to: AT+B" + (String)baud);
  #endif
  return 0;
}

rc_status_t RadioCommunication::sendBytes()
{
  return TRANSMIT_SUCCES;
}

rc_status_t RadioCommunication::parseIncomingBytes()
{
  #ifdef TARGET_TEENSY35
  if (HC12.available() > 0 && HC12.peek() == (DATA_BYTES || CONFIG_BYTES))
  {
    if (packet[0] == DATA_BYTES) // data parsing mode
    {
      for (i = 0; i < 15; i++)
      {
        packet[i] = HC12.read();
        Serial.println(packet[i]);
      }
      // checksum
      local_checksum = 0;
      for (int i = 1; i < 13; i++)
      {
        local_checksum += packet[i];
      }
      pre_checksum = packet[13] | packet[14] >> 8;
      Serial.print("local check: ");
      Serial.print(local_checksum);
      Serial.print("  pre check: ");
      Serial.println(pre_checksum);

      if (pre_checksum == local_checksum)
      {
        rc_button1 = packet[1];
        rc_button2 = packet[2];
        rc_button3 = packet[3];
        rc_button4 = packet[4];

        rc_throttle = packet[6] << 8 | packet[5];
        rc_yaw = packet[8] << 8 | packet[7];
        rc_pitch = packet[10] << 8 | packet[9];
        rc_roll = packet[12] << 8 | packet[11];
        Serial.println((String) "P1: " + rc_button1 + "  P2: " + rc_button2 + "  P3: " + rc_button3 + "  P4: " + rc_button4);
        Serial.println((String) "AY: " + rc_throttle + "  AX: " + rc_yaw + "  BY: " + rc_pitch + "  BX: " + rc_roll);
        Serial.println(" ");
        return RECEIVE_SUCCES;
      }
      else
      {
        return RECEIVE_FAILED;
      }
    }

    if (packet[0] == CONFIG_BYTES)
    {
      for (i = 0; i < 6; i++)
      {
        packet[i] = HC12.read();
        Serial.println(packet[i]);
      }

      local_checksum = packet[1] + packet[2] + packet[3] + packet[4];
      pre_checksum = packet[6];

      if (pre_checksum == local_checksum)
      {
        if (packet[1] != 0) // channel setting
        {
          setChannel(packet[1]);
        }
        if (packet[2] != 0) // transmit power setting
        {
          setTxPower(rc_txpower[packet[2]]);
        }
        if (packet[3] != 0) // baudrate setting
        {
          setBaudRate(rc_baudrate[(packet[3])]);
        }
        if (packet[4] != 0) 
        {
          
        }
        return RECEIVE_SUCCES;
      }
      else
      {
        return RECEIVE_FAILED;
      }
    }
  }
  else
  {
    HC12.clear();
    return RECEIVE_FAILED;
  }
  #endif
}
