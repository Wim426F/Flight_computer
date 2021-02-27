#include <globals.h>

#define PWB_ADDR_IIC 1 // Powerboard address

/* Powerboard Variables */
byte pwb_data[21];
float motor1_amps = 0;
float motor2_amps = 0;
float battery_amps = 0;
float battery_temp = 0;
float battery_voltage = 0;
byte pwb_status = 0;

void fetchPwb()
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
    } current_tot_u;
    current_tot_u.bytes[0] = pwb_data[0];
    current_tot_u.bytes[1] = pwb_data[1];
    current_tot_u.bytes[2] = pwb_data[2];
    current_tot_u.bytes[3] = pwb_data[3];
    battery_amps = current_tot_u.fvalue * 60;

    union Param2
    {
      byte bytes[4];
      float fvalue;
    } DRPS_Union; //DRPS = Drum Revs per Second
    DRPS_Union.bytes[0] = pwb_data[4];
    DRPS_Union.bytes[1] = pwb_data[5];
    DRPS_Union.bytes[2] = pwb_data[6];
    DRPS_Union.bytes[3] = pwb_data[7];
    motor1_amps = DRPS_Union.fvalue;
  }

  Wire.endTransmission();
}

void powerMonitor() // Main function
{
  fetchPwb();
}
