#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

void initController();
void mainControl();
void enableWaypoint(float longitude, float latitude, float altitude);

extern bool hold_position;
extern float hold_altitude;

extern uint16_t left_front;  // M2: CW
extern uint16_t left_side;   // M3: CCW
extern uint16_t left_rear;   // M4: CW
extern uint16_t right_front; // M1: CCW
extern uint16_t right_side;  // M6: CW
extern uint16_t right_rear;  // M5: CCW

extern float total_angle_x;
extern float total_angle_y;
extern float total_angle_z;

extern float p_gain;
extern float i_gain;
extern float d_gain;
extern float angle_x_setpoint;
extern float angle_y_setpoint;
extern float angle_z_setpoint;

extern int16_t acc_x, acc_y, acc_z;
extern int16_t gyr_x, gyr_y, gyr_z;
extern int16_t mag_x, mag_y, mag_z;



#endif
