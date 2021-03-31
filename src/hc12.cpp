#include <string>
#include <hc12.h>
#include <pw_monitor.h>
#include <globals.h>

using namespace std;
String set_baudrate;

// receiver parameters are stored in eeprom
uint8_t last_channel = EEPROM.read(CH_STATE_ADDR);
uint8_t last_baud = EEPROM.read(BD_STATE_ADDR);
uint8_t last_txpower = EEPROM.read(TP_STATE_ADDR);
uint8_t last_txmode = EEPROM.read(TM_STATE_ADDR);

const float rc_txpower[8] = {0.8, 1.6, 3.2, 6.3, 12, 25, 50, 100};
const uint32_t rc_baudrate[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};

uint8_t packet[25];
uint8_t send_packet[25];

HC12::rc_ack_t HC12::setChannel(int new_channel)
{
#ifdef TARGET_TEENSY35
  ACK ack;
  if (new_channel != last_channel)
  {
    String set_channel;
    if (new_channel < 10)
    {
      set_channel = "AT+C00" + (String)new_channel;
    }
    if (new_channel >= 10 && new_channel < 100)
    {
      set_channel = "AT+C0" + (String)new_channel;
    }
    if (new_channel >= 100)
    {
      set_channel = "AT+C" + (String)new_channel;
    }

    digitalWrite(HC12_CMD_MODE, LOW); // Then change transmitter param
    delay(40);

    hc12_uart.println(set_channel);
    Serial.println(set_channel);

    String feedback = hc12_uart.readString();
    if (feedback.indexOf("OK+C") != -1)
    {
      Serial.print("Succes! : ");
      Serial.println(feedback);
      EEPROM.write(CH_STATE_ADDR, new_channel);
      last_channel = EEPROM.read(CH_STATE_ADDR);
      ack = ACK::OK;
    }
    else
    {
      Serial.print("Failed! : ");
      Serial.println(feedback);
      ack = ACK::CMD_FAILED;
    }
    digitalWrite(HC12_CMD_MODE, HIGH);
    delay(80);
  }
  else
  {
    ack = ACK::OK;
    Serial.println((String) "Channel is already " + new_channel);
  }
#endif

  return ack;
}

HC12::rc_ack_t HC12::setTxPower(int new_txpower)
{
#ifdef TARGET_TEENSY35
  ACK ack;
  if (new_txpower != last_txpower)
  {
    digitalWrite(HC12_CMD_MODE, LOW);
    delay(40);

    Serial.println((String) "AT+P" + (new_txpower));
    hc12_uart.println((String) "AT+P" + (new_txpower));

    String feedback = hc12_uart.readString();
    if (feedback.indexOf("OK+P") != -1)
    {
      Serial.print("Succes! : ");
      Serial.println(feedback);
      EEPROM.write(TP_STATE_ADDR, new_txpower);
      last_txpower = EEPROM.read(TP_STATE_ADDR);
      Serial.print((String)rc_txpower[new_txpower - 1] + "mW");
      ack = ACK::OK;
    }
    else
    {
      Serial.print("Failed! : ");
      Serial.println(feedback);
      ack = ACK::CMD_FAILED;
    }

    digitalWrite(HC12_CMD_MODE, HIGH);
    delay(80);
  }
  else
  {
    ack = ACK::OK;
    Serial.println((String) "Transmit power is already " + rc_txpower[new_txpower - 1] + "mW");
  }
#endif
  return ack;
}

HC12::rc_ack_t HC12::setBaudRate(int new_baud)
{
#ifdef TARGET_TEENSY35
  ACK ack;
  digitalWrite(HC12_CMD_MODE, LOW);
  delay(40);

  hc12_uart.println("AT+B" + (String)rc_baudrate[new_baud]);
  Serial.println("AT+B" + (String)rc_baudrate[new_baud]);

  String feedback = hc12_uart.readString();
  if (feedback.indexOf("OK+B") != -1)
  {
    Serial.print("Succes! : ");
    Serial.println(feedback);
    EEPROM.write(BD_STATE_ADDR, new_baud);
    last_baud = EEPROM.read(BD_STATE_ADDR);

    hc12_uart.end();
    hc12_uart.begin(rc_baudrate[last_baud], SERIAL_8N1);
    ack = ACK::OK;
  }
  else
  {
    Serial.print("Failed! : ");
    Serial.println(feedback);
    ack = ACK::OK;
  }

  digitalWrite(HC12_CMD_MODE, HIGH);
  delay(80);

#endif
  return ack;
}

HC12::rc_ack_t HC12::setTransmitMode(int new_mode)
{
  ACK ack;
  if (new_mode != last_txmode)
  {
    digitalWrite(HC12_CMD_MODE, LOW);
    delay(40);
    hc12_uart.println("AT+FU" + (String)new_mode);
    Serial.println("AT+FU" + (String)new_mode);

    String feedback = hc12_uart.readString();
    if (feedback.indexOf("OK+FU") != -1)
    {
      if (new_mode == 4)
      {
        Serial.println("mode 4: baudrate set to 1200bps");
        EEPROM.write(BD_STATE_ADDR, 0); // set baudrate to 1200bps
        last_baud = EEPROM.read(BD_STATE_ADDR);
        hc12_uart.end();
        hc12_uart.begin(rc_baudrate[last_baud], SERIAL_8N1);
      }

      if (new_mode == 2)
      {
        if (rc_baudrate[last_baud] > 4800)
        {
          Serial.println("mode 2: baudrate set to 4800bps");
          EEPROM.write(BD_STATE_ADDR, 2); // set baudrate to 4800bps
          last_baud = EEPROM.read(BD_STATE_ADDR);
          hc12_uart.end();
          hc12_uart.begin(rc_baudrate[last_baud], SERIAL_8N1);
        }
      }

      Serial.print("Succes! : ");
      Serial.println(feedback);
      EEPROM.write(TM_STATE_ADDR, new_mode);
      last_txmode = EEPROM.read(TM_STATE_ADDR);
      ack = ACK::OK;
    }
    else
    {
      Serial.print("Failed! : ");
      Serial.println(feedback);
      ack = ACK::CMD_FAILED;
    }

    digitalWrite(HC12_CMD_MODE, HIGH);
    delay(80);
  }
  else
  {
    Serial.println((String) "Transmit Mode is already " + new_mode);
    ack = ACK::OK;
  }
  return ack;
}

HC12::rc_ack_t HC12::setAllDefault()
{
  ACK ack;
  digitalWrite(HC12_CMD_MODE, LOW);
  delay(40);
  hc12_uart.println("AT+DEFAULT");
  Serial.println("AT+DEFAULT");

  String feedback = hc12_uart.readString();
  if (feedback.indexOf("OK+DEFAULT") != -1)
  {
    Serial.print("Succes! : ");
    Serial.println(feedback);
    Serial.println("channel: 1, baud: 9600, txpower: 100mw, transmit_mode: FU3");

    EEPROM.write(CH_STATE_ADDR, 1); // channel 1
    EEPROM.write(BD_STATE_ADDR, 3); // baudrate of 9600
    EEPROM.write(TP_STATE_ADDR, 8); // 100mW
    EEPROM.write(TM_STATE_ADDR, 3); // FU3

    last_channel = EEPROM.read(CH_STATE_ADDR);
    last_baud = EEPROM.read(BD_STATE_ADDR);
    last_txpower = EEPROM.read(TP_STATE_ADDR);
    last_txmode = EEPROM.read(TM_STATE_ADDR);

    hc12_uart.end();
    hc12_uart.begin(rc_baudrate[last_baud], SERIAL_8N1);
    ack = ACK::OK;
  }
  else
  {
    ack = ACK::CMD_FAILED;
    Serial.print("Failed! : ");
    Serial.println(feedback);
  }

  digitalWrite(HC12_CMD_MODE, HIGH);
  delay(80);
  return ack;
}

void HC12::getVersion()
{
  digitalWrite(HC12_CMD_MODE, LOW);
  delay(40);
  hc12_uart.println("AT+V");

  String feedback = hc12_uart.readString();

  if (feedback.indexOf("HC") != -1)
  {
    Serial.print("Firmware version : ");
    Serial.println(feedback);
  }
  else
  {
    Serial.print("Get version failed! : ");
    Serial.println(feedback);
  }

  digitalWrite(HC12_CMD_MODE, HIGH);
  delay(80);
}

void HC12::sleep()
{
  digitalWrite(HC12_CMD_MODE, LOW);
  delay(40);
  hc12_uart.println("AT+SLEEP");

  String feedback = hc12_uart.readString();
  if (feedback.indexOf("OK+SLEEP") != -1)
  {
    Serial.println("HC12 entered sleep mode");
  }
  else
  {
    Serial.print("HC12 not sleepy! : ");
    Serial.println(feedback);
  }

  digitalWrite(HC12_CMD_MODE, HIGH);
  delay(80);
}

void HC12::wakeUp()
{
  digitalWrite(HC12_CMD_MODE, LOW);
  delay(40);

  hc12_uart.println("AT");
  String feedback = hc12_uart.readString();

  if (feedback.indexOf("OK") != -1)
  {
    Serial.println("HC12 exited sleep mode");
  }
  else
  {
    Serial.print("HC12 didn't wake up! : ");
    Serial.println(feedback);
  }
  digitalWrite(HC12_CMD_MODE, HIGH);
  delay(80);
}

void HC12::sendData(enum HC12::ACK ack)
{
  temperature_icp *= 100;
  batt_temp = (int)temperature_icp;
  temperature_icp /= 100;

  //Serial.print("Answer: ");
  send_packet[0] = RECEIVER_ANSWER;
  send_packet[1] = ack;
  send_packet[2] = 6;

  // battery voltage
  send_packet[3] = batt_voltage >> 8;
  send_packet[4] = batt_voltage & 0xff;
  // battery temperature
  send_packet[5] = batt_temp >> 8;
  send_packet[6] = batt_temp & 0xff;
  // battery current
  send_packet[7] = batt_amps >> 8;
  send_packet[8] = batt_amps & 0xff;

  uint16_t crc = crc_xmodem(send_packet, 9); //crc
  send_packet[9] = crc >> 8;                 // Low byte
  send_packet[10] = crc & 0xff;              // High byte

  for (int i = 0; i < 11; i++)
  {
    //Serial.print(send_packet[i]);
    //Serial.print(" ");
    hc12_uart.write(send_packet[i]);
    hc12_uart.flush();
    long flush_time = 1000000 / (rc_baudrate[last_baud] / 8);
    delayMicroseconds(flush_time);
  }
}

HC12::rc_ack_t HC12::readData()
{
#ifdef TARGET_TEENSY35
  ACK ack;
  if (hc12_uart.available() > 0 && hc12_uart.peek() != 0)
  {
    //Serial.print("\n\nbytes arrived: ");

    byte incoming_bytes = 0;

    for (int i = 0; hc12_uart.available() > 0; i++)
    {
      long serial_delay = 1000000 / (rc_baudrate[last_baud] / 8) * 1.5;
      delayMicroseconds(serial_delay); // wait for bytes to enter buffer
      packet[i] = hc12_uart.read();
      incoming_bytes++;
      //Serial.print(packet[i]);
      //Serial.print(" ");
    }

    //Serial.println("");

    if (packet[0] == TRANSMITTER_CMD)
    {
      uint16_t local_crc = crc_xmodem(packet, incoming_bytes - 2);
      uint16_t transmitter_crc = packet[incoming_bytes - 2] << 8 | packet[incoming_bytes - 1];

      if (local_crc == transmitter_crc) // verify crc
      {
        //Serial.println("CRC correct: " + (String)transmitter_crc);
        if (packet[1] == MODE::NORMAL)
        {
          if (packet[2] >= 4) // data length
          {
            rc_throttle = packet[3];
            rc_yaw = packet[4];
            rc_pitch = packet[5];
            rc_roll = packet[6];
            //rc_pitch = packet[10] << 8 | packet[9];
            Serial.print((String) "AY: " + rc_throttle + "  AX: " + rc_yaw + "  BY: " + rc_pitch + "  BX: " + rc_roll);
            Serial.println(" ");
          }
          if (packet[2] >= 8) // data length
          {
            rc_param1 = packet[7];
            rc_param2 = packet[8];
            rc_param3 = packet[9];
            rc_param4 = packet[10];
            Serial.print((String) "P1: " + rc_param1 + "  P2: " + rc_param2 + "  P3: " + rc_param3 + "  P4: " + rc_param4);
            Serial.println(" ");
          }
          ack = ACK::OK;
          sendData(ack);
        }

        if (packet[1] == MODE::SET_CHANNEL)
        {
          delay(2000);
          ack = setChannel(packet[3]); // packet 2 is not used
          sendData(ack);
        }

        if (packet[1] == MODE::SET_BAUD)
        {
          delay(2000);
          ack = setBaudRate(packet[3]);
          sendData(ack);
        }

        if (packet[1] == MODE::SET_MODE)
        {
          delay(2000);
          ack = setTransmitMode(packet[3]);
          sendData(ack);
        }

        if (packet[1] == MODE::SET_TXPOWER)
        {
          ack = setTxPower(packet[3]);
          sendData(ack);
        }

        if (packet[1] == MODE::SET_DEFAULT)
        {
          delay(2000);
          ack = setAllDefault();
          sendData(ack);
        }
        //Serial.print("CRC OK");
      }
      else
      {
        Serial.println("CRC failed");
        ack = ACK::INVALID_CRC;
        sendData(ack);
      }
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_RED, LOW);
    }
    else
    {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, HIGH);
      ack = ACK::CMD_INVALID;
      sendData(ack);
      Serial.println("Command invalid\n\r");
    }
  }
  else
  {
    digitalWrite(LED_GREEN, LOW);
    ack = ACK::NOT_READY;
  }
  return ack;
#endif
}
