#include <string>
#include <sensors.h>
#include <globals.h>

using namespace std;

sensor_status_t sensors_init()
{
    #ifdef TARGET_TEENSY35
    /* I2C */
    Wire.setClock(400000);
    Wire.begin(0x02); // Set address of this device
    bmp.begin(0x76);
    bmp.begin(); // Address 0x68
    groundlvl_pressure = bmp.readPressure() / 100; // set altitude to zero relative to air pressure

    pinMode(SONAR_TRIGGER, OUTPUT); // Trigger pin for HC-SR04
    pinMode(SONAR_ECHO, INPUT);     // Echo pin for HC-SR04
    #endif

    return SN_INIT_SUCCES;
}

sensor_status_t sensors_update(string sensor)
{
    #ifdef TARGET_TEENSY35
    bmp_pressure = bmp.readPressure() / 100;
    bmp_altitude = bmp.readAltitude(groundlvl_pressure);
    #endif
    
    return SN_UPDATE_SUCCES;
}

pwb_status_t pwb_initialize()
{
    return PWB_INIT_SUCCES;
}

pwb_status_t pwb_fetch(Pwb param)
{
    if (param == ALL)
    {
    }
    return PWB_FETCH_SUCCES;
}

int get_sonar_distance()
{
    uint8_t distance = 0;
    return distance;
}
