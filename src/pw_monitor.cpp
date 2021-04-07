#include <globals.h>
#include <sensors.h>

#define PWB_ADDR_IIC 1 // Powerboard address

/* Powerboard Variables */
byte pwb_data[21];
float motor1_amps = 0;
float motor2_amps = 0;
uint16_t batt_amps = 6374;
uint16_t batt_temp = 2345;
uint16_t batt_voltage = 16878;
byte voltage_detect = 0;
byte pwb_status = 0;

float used_capacity = 0;

const float MAX_BATT_TEMP = 45; // Celsius
const float MIN_VOLTAGE_3S = 9;
const float MIN_VOLTAGE_4S = 11.2;
const float MAX_CURRENT_3S = 100; // 17A per motor
const float MAX_CURRENT_4S = 150; // 25A per motor

void updatePwb()
{
  Wire.beginTransmission(PWB_ADDR_IIC);
  Wire.requestFrom(PWB_ADDR_IIC, 24);

  if (Wire.available())
  {
    int i = 0;
    while (Wire.available()) // slave may send less than requested
    {
      pwb_data[i] = Wire.receive(); // receive a byte as character
      i = i + 1;
    }

    //A union datatypes makes the byte and float elements share the same piece of memory, which enables conversion from a byte array to a float possible
    union Param1
    {
      byte bytes[4];
      float fvalue;
    } param1;
    param1.bytes[0] = pwb_data[0];
    param1.bytes[1] = pwb_data[1];
    param1.bytes[2] = pwb_data[2];
    param1.bytes[3] = pwb_data[3];
    batt_amps = param1.fvalue * 60;

    union Param2
    {
      byte bytes[4];
      float fvalue;
    } param2;
    param2.bytes[0] = pwb_data[4];
    param2.bytes[1] = pwb_data[5];
    param2.bytes[2] = pwb_data[6];
    param2.bytes[3] = pwb_data[7];
    motor1_amps = param2.fvalue;

    union Param3
    {
      byte bytes[4];
      float fvalue;
    } param3;
    param3.bytes[0] = pwb_data[4];
    param3.bytes[1] = pwb_data[5];
    param3.bytes[2] = pwb_data[6];
    param3.bytes[3] = pwb_data[7];
    motor2_amps = param3.fvalue;

    union Param4
    {
      byte bytes[4];
      float fvalue;
    } param4;
    param4.bytes[0] = pwb_data[4];
    param4.bytes[1] = pwb_data[5];
    param4.bytes[2] = pwb_data[6];
    param4.bytes[3] = pwb_data[7];
    batt_voltage = param4.fvalue;

    union Param5
    {
      byte bytes[4];
      float fvalue;
    } param5;
    param5.bytes[0] = pwb_data[4];
    param5.bytes[1] = pwb_data[5];
    param5.bytes[2] = pwb_data[6];
    param5.bytes[3] = pwb_data[7];
    batt_temp = param5.fvalue;
  }

  Wire.endTransmission();
}

void powerMonitor() // Main function
{
  updatePwb();

  if (batt_voltage < MIN_VOLTAGE_3S && batt_amps < 1) // when in standby
  {
#ifdef TARGET_TEENSY35
    hc12.sleep();
#endif
  }
}
