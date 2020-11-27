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

int RadioCommunication::setChannel(int channel)
{
  String set_channel;
  if (channel < 10)
  {
    set_channel = "AT+C00" + (String)channel;
  }
  else
  {
    set_channel = "AT+C0" + (String)channel;
  }

  digitalWrite(hc_set, LOW);
  delay(40);
  HC12.println(set_channel);
  delay(20);
  digitalWrite(hc_set, HIGH);
  delay(80);
  Serial.println("channel set");
  return 0;
}
int RadioCommunication::setTxPower(int power)
{
  digitalWrite(hc_set, LOW);
  delay(40);
  HC12.println((String) "AT+P" + power);
  delay(20);
  digitalWrite(hc_set, HIGH);
  delay(80);
  Serial.println("power set");
  return 0;
}
int RadioCommunication::setBaudRate(int baud)
{
  digitalWrite(hc_set, LOW);
  delay(40);
  HC12.println("AT+B" + (String)baud);
  HC12.end();
  delay(20);
  digitalWrite(hc_set, HIGH);
  delay(80);

  HC12.begin(baud);
  delay(50);
  Serial.println("baudrate set to: AT+B" + (String)baud);
  return 0;
}

rc_status_t RadioCommunication::transmit()
{
  return TRANSMIT_SUCCES;
}

rc_status_t RadioCommunication::receive()
{
  for (i = 0; i < 9 && HC12.available() > 0; i++)
  {
    packet[i] = HC12.read();
    delayMicroseconds(1);
    Serial.println(packet[i]);
  }
  HC12.clear();
  // checksum (average number of all values)
  uint16_t local_check = (packet[0] + packet[1] + packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7])/8;
  uint8_t pre_check = packet[8];
  Serial.print("local check: ");
  Serial.print(local_check);
  Serial.print("  pre check: ");
  Serial.println(pre_check);
  Serial.println(" ");

  if (pre_check == local_check)
  {
    if (packet[0] != 255) // if first byte is binary then update controls
    {
      rc_button1 = packet[0];
      rc_button2 = packet[1];
      rc_button3 = packet[2];
      rc_button4 = packet[3];

      rc_throttle = packet[4];
      rc_yaw = packet[5];
      rc_pitch = packet[6];
      rc_roll = packet[7];
      Serial.println((String) "P1: " + rc_button1 + "  P2: " + rc_button2 + "  P3: " + rc_button3 + "  P4: " + rc_button4);
      Serial.println((String) "AY: " + rc_throttle + "  AX: " + rc_yaw + "  BY: " + rc_pitch + "  BX: " + rc_roll);
      Serial.println(" ");
    }
    if (packet[0] == 255) // if first byte is 255, change parameters
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
      if (packet[4] != 0) // steering correction %
      {
        correction_multiplier = packet[4];
      }
    }
    return RECEIVE_SUCCES;
  }
  else
  {
    return RECEIVE_FAILED;
  }
}
