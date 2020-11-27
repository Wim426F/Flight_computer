#include <string>
#include <sensors.h>
#include <globals.h>

using namespace std;

sensor_status_t sensors_init()
{
    /* I2C */
    Wire.setClock(400000);
    Wire.begin(0x02); // Set address of this computer
    bmp.begin(0x76);
    mpu6050.begin(); // Address 0x68
    mpu6050.calcGyroOffsets(true);
    groundlvl_pressure = bmp.readPressure() / 100; // set altitude to zero relative to air pressure

    pinMode(sonar_trigg_pin, OUTPUT); // Trigger pin for HC-SR04
    pinMode(sonar_echo_pin, INPUT);     // Echo pin for HC-SR04
    return SN_INIT_SUCCES;
}

sensor_status_t sensors_update(string sensor)
{
    mpu6050.update();
    bmp_pressure = bmp.readPressure() / 100;
    bmp_altitude = bmp.readAltitude(groundlvl_pressure);
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
