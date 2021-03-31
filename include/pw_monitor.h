#ifndef PW_MONITOR_H
#define PW_MONITOR_H


void powerMonitor();

/* Powerboard Variables */
extern float motor1_amps;
extern float motor2_amps;
extern uint16_t batt_amps;
extern uint16_t batt_temp;
extern uint16_t batt_voltage;

#endif