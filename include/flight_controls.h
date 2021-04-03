#ifndef FLIGHT_CONTROLS_H
#define FLIGHT_CONTROLS_H


void initController();
void calibrateAngles();
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

extern int16_t acc_x, acc_y, acc_z;
extern int16_t gyr_x, gyr_y, gyr_z;
extern int16_t mag_x, mag_y, mag_z;



#endif
