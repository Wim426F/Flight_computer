#ifndef SENSORS_H_
#define SENSORS_H_
#include <Arduino.h>
#include <string>

typedef enum
{
    SN_INIT_SUCCES = 0,
    SN_INIT_FAILED = 1,
    SN_UPDATE_SUCCES = 2,
    SN_UPDATE_FAILED = 3,
} sensor_status_t;

typedef enum
{
    PWB_INIT_SUCCES = 0,
    PWB_INIT_FAILED = 1,
    PWB_FETCH_SUCCES = 2,
    PWB_FETCH_FAILED = 3,
} pwb_status_t;

enum Pwb
{
    VOLTAGE_TOTAL = 0,
    VOLTAGE_CELL1 = 1,
    VOLTAGE_CELL2 = 2,
    CURRENT_TOTAL = 3,
    CURRENT_M1 = 4,
    CURRENT_M2 = 5,
    ALL = 6
};

sensor_status_t sensors_init();
sensor_status_t sensors_update(std::string sensor = "");

pwb_status_t pwb_init();
pwb_status_t pwb_fetch(Pwb param);

int get_sonar_distance();

#endif