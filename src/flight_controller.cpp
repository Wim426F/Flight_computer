#include <math.h>
#include <Arduino.h>
#include <flight_controller.h>
#include <globals.h>
#include <sensors.h>

const int PWM_FREQ = 4000; // oneshot42: 12000Hz, servo: 50Hz, oneshot125: 4000Hz
const int PWM_RES = 11;
const int THROTTLE_MIN = 1024, THROTTLE_MAX = 2048;

bool hold_position = false;
float hold_altitude = 0;

double elapsed_time, current_time, previous_time;

uint16_t left_front = 0;  // M2: CW
uint16_t left_side = 0;   // M3: CCW
uint16_t left_rear = 0;   // M4: CW
uint16_t right_front = 0; // M1: CCW
uint16_t right_side = 0;  // M6: CW
uint16_t right_rear = 0;  // M5: CCW

int yaw = 0, pitch = 0, roll = 0, throttle = 0;
int steer_diff = 0; // only allow --% of throttle to be shifted

int acc_x = 0, acc_y = 0, acc_z = 0;
int gyr_x = 0, gyr_y = 0, gyr_z = 0;
int mag_x = 0, mag_y = 0, mag_z = 0;

float total_angle_x = 0; // roll
float total_angle_y = 0; // pitch
float total_angle_z = 0; // yaw

float mag_heading = 0;       // yaw calculated w/magnetometer
float declination_angle = 2; // depends on location
float rad_to_deg = 180 / 3.141592654f;

/* PID */
float angle_x_setpoint = 0;
float angle_y_setpoint = 0;
float angle_z_setpoint = 0;

float max_angle_x = 40.0f;
float max_angle_y = 40.0f;
float max_yaw_speed_z = 250.0f; // dps

float p_gain = 3.5f;
float i_gain = 0.0005f;
float d_gain = 1.27f;

float error_x = 0;
float error_y = 0;
float error_z = 0;
float error_alt = 0;

float prev_error_x = 0;
float prev_error_y = 0;
float prev_error_z = 0;
float prev_error_alt = 0;

float p_term_x = 0;
float p_term_y = 0;
float p_term_z = 0;
float p_term_alt = 0;

float i_term_x = 0;
float i_term_y = 0;
float i_term_z = 0;
float i_term_alt = 0;

float d_term_x = 0;
float d_term_y = 0;
float d_term_z = 0;
float d_term_alt = 0;

float pid_x = 0;
float pid_y = 0;
float pid_z = 0;
float pid_alt = 0;

void initController()
{
  //Configure ESC's
  analogWriteResolution(PWM_RES);         // 12 bit (0 - 4096)
  analogWriteFrequency(MOTOR1, PWM_FREQ); // FTM0 timer, pin 5, 6
  analogWriteFrequency(MOTOR5, PWM_FREQ); // FTM1 timer, pin 4
  analogWriteFrequency(MOTOR2, PWM_FREQ); // FTM3 timer, pin 7, 8 ,14

  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT);
  pinMode(MOTOR4, OUTPUT);
  pinMode(MOTOR5, OUTPUT);
  pinMode(MOTOR6, OUTPUT);
  analogWrite(MOTOR1, THROTTLE_MIN);
  analogWrite(MOTOR2, THROTTLE_MIN);
  analogWrite(MOTOR3, THROTTLE_MIN);
  analogWrite(MOTOR4, THROTTLE_MIN);
  analogWrite(MOTOR5, THROTTLE_MIN);
  analogWrite(MOTOR6, THROTTLE_MIN);
}

void getTotalAngles()
{
  updateImu();
  //updateMagnetoMeter();

  // complementary filters
  float acc_angle_x = atan(-1 * (acc_x / 16384.0) / sqrt(pow((acc_y / 16384.0), 2) + pow((acc_z / 16384.0), 2))) * rad_to_deg;
  float acc_angle_y = atan((acc_y / 16384.0) / sqrt(pow((acc_x / 16384.0), 2) + pow((acc_z / 16384.0), 2))) * rad_to_deg;

  float gyr_angle_x = (gyr_x / 65.5) * elapsed_time;
  float gyr_angle_y = (gyr_y / 65.5) * elapsed_time;
  float gyr_angle_z = (gyr_z / 65.5) * elapsed_time;

  total_angle_x = 0.98 * (total_angle_x + gyr_angle_x) + 0.02 * acc_angle_x;
  total_angle_y = 0.98 * (total_angle_y + gyr_angle_y) + 0.02 * acc_angle_y;
  total_angle_z = gyr_angle_z;

  float mag_angle_x = total_angle_x;                       // roll
  float mag_angle_y = total_angle_y - (total_angle_y * 2); // mag and gyr axis are opposed
  float mag_angle_z = gyr_angle_z - (gyr_angle_z * 2);

  float mag_x_hor = mag_x * cos(mag_angle_y) + mag_y * sin(mag_angle_x) * sin(mag_angle_y) - mag_z * cos(mag_angle_x) * sin(mag_angle_y);
  float mag_y_hor = mag_y * cos(mag_angle_x) + mag_z * sin(mag_angle_x);
  mag_heading = (atan2(mag_x_hor, mag_y_hor) * rad_to_deg) + declination_angle;

  Serial.print("  X:");
  Serial.print(total_angle_x, 2);

  //Serial.print("  Y:");
  //Serial.print(total_angle_y, 2);

  //Serial.print("  Z:");
  //Serial.print(total_angle_z, 2);
}

void correct_pitch(int pid_input = 0)
{
  //Serial.print("   pitch: " + (String)pid_input);

  left_front -= pid_input;
  right_front -= pid_input;
  left_rear += pid_input;
  right_rear += pid_input;
}

void correct_roll(int pid_input = 0)
{
  //Serial.println("   roll: " + (String)correction);

  left_side -= pid_input;
  left_front -= pid_input * 0.75;
  left_rear -= pid_input * 0.75;
  right_side += pid_input;
  right_front += pid_input * 0.75;
  right_rear += pid_input * 0.75;
}

void correct_yaw(int pid_input = 0)
{
  int correction = map(yaw, -1023, 1023, -steer_diff, steer_diff);
  correction += pid_input;

  //Serial.println("   yaw+: " + (String)correction);

  right_front -= correction;
  right_rear -= correction;
  left_side -= correction;
  left_front += correction;
  left_rear += correction;
  right_side += correction;
}

void compute_pid()
{
  /* Error from setpoint */
  error_x = total_angle_x - angle_x_setpoint;
  error_y = total_angle_y - angle_x_setpoint;
  error_z = total_angle_z - angle_x_setpoint;
  error_alt = relative_altitude - hold_altitude;

  /* Proportional correction */
  p_term_x = error_x * p_gain;
  p_term_y = error_y * p_gain;
  p_term_z = error_z * p_gain;
  p_term_alt = error_alt * p_gain;

  /* Integral correction */
  i_term_x += error_x * i_gain;
  i_term_y += error_y * i_gain;
  i_term_z += error_z * i_gain;
  i_term_alt += error_alt * i_gain;

  /* Derivative correction */
  d_term_x = ((error_x - prev_error_x) / elapsed_time) * d_gain;
  d_term_y = ((error_y - prev_error_y) / elapsed_time) * d_gain;
  d_term_z = ((error_z - prev_error_z) / elapsed_time) * d_gain;
  d_term_alt = ((error_alt - prev_error_alt) / elapsed_time) * d_gain;

  pid_x = p_term_x + i_term_x + d_term_x;
  pid_y = p_term_y + i_term_y + d_term_y;
  pid_z = p_term_z + i_term_z + d_term_z;
  pid_alt = p_term_alt + i_term_alt + d_term_alt;

  pid_alt = constrain(pid_alt, -steer_diff, steer_diff); // throttle
  //pid_x = constrain(pid_x, -steer_diff, steer_diff);

  prev_error_x = error_x;
  prev_error_y = error_y;
  prev_error_z = error_z;
  prev_error_alt = error_alt;
}

void mainControl()
{
  getTotalAngles();

  previous_time = current_time;
  current_time = micros();
  elapsed_time = (current_time - previous_time) / 1000000;

  /*   
    steering is always max 50% of current throttle.
    steering levers are only 8 bits and thus scaled up. PID uses the full range.
  */

  throttle = map(rc_throttle, 0, 255, THROTTLE_MIN, THROTTLE_MAX);
  steer_diff = (THROTTLE_MAX - THROTTLE_MIN) / 2;

  left_front = left_side = left_rear = right_front = right_side = right_rear = throttle;

  angle_x_setpoint = map(roll, 0, 255, -max_angle_x, max_angle_x);
  angle_y_setpoint = map(pitch, 0, 255, -max_angle_y, max_angle_y);
  max_yaw_speed_z = map(yaw, 0, 255, -max_yaw_speed_z, max_yaw_speed_z);

  compute_pid();

  Serial.print((String) "   Px: " + p_term_x);
  Serial.print((String) "   Ix: " + i_term_x);
  Serial.print((String) "   Dx: " + d_term_x);
  Serial.print((String) "   gain p " + p_gain);
  Serial.print((String) "   gain i ");
  Serial.print(i_gain, 5);
  Serial.print((String) "   gain d " + d_gain);
  Serial.print((String) "   setp " + angle_x_setpoint);

  correct_roll(pid_x);
  correct_pitch(pid_y);
  correct_yaw(pid_z);

  Serial.println("");

  if (throttle < (THROTTLE_MIN + 50))
  {
    left_front = left_side = left_rear = right_front = right_side = right_rear = 0;
  }

  left_front = constrain(left_front, THROTTLE_MIN, THROTTLE_MAX);
  left_side = constrain(left_side, THROTTLE_MIN, THROTTLE_MAX);
  left_rear = constrain(left_rear, THROTTLE_MIN, THROTTLE_MAX);
  right_front = constrain(right_front, THROTTLE_MIN, THROTTLE_MAX);
  right_side = constrain(right_side, THROTTLE_MIN, THROTTLE_MAX);
  right_rear = constrain(right_rear, THROTTLE_MIN, THROTTLE_MAX);

  analogWrite(MOTOR1, right_front);
  analogWrite(MOTOR2, left_front);
  analogWrite(MOTOR3, left_side);
  analogWrite(MOTOR4, left_rear);
  analogWrite(MOTOR5, right_rear);
  analogWrite(MOTOR6, right_side);
}

void avoidObstacles()
{
}

void enableWaypoint(float longitude, float latitude, float altitude)
{
}
