#ifndef PW_MONITOR_H_
#define PW_MONITOR_H_


void powerMonitor();

/* Powerboard Variables */
extern float motor1_amps;
extern float motor2_amps;
extern float battery_amps;
extern float battery_temp;
extern float battery_voltage;

#endif