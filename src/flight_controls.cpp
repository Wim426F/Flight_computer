#include <Arduino.h>
#include <math.h>

#include <flight_controls.h>
#include <globals.h>
#include <sensors.h>

const float PWM_FREQ = 50;
const uint16_t THROTTLE_MIN = 2800, THROTTLE_MAX = 6800;

bool hold_position = false;
float hold_altitude = 0;

double elapsed_time, current_time, previous_time;

uint16_t left_front = 0;  // M2: CW
uint16_t left_side = 0;   // M3: CCW
uint16_t left_rear = 0;   // M4: CW
uint16_t right_front = 0; // M1: CCW
uint16_t right_side = 0;  // M6: CW
uint16_t right_rear = 0;  // M5: CCW

int yaw = 0;
int pitch = 0;
int roll = 0;
int throttle = 0;
int steer_diff = 0; // only allow 50% of throttle to be shifted

int16_t acc_x = 0, acc_y = 0, acc_z = 0;
int16_t gyr_x = 0, gyr_y = 0, gyr_z = 0;
int16_t mag_x = 0, mag_y = 0, mag_z = 0;

float total_angle_x = 0; // roll
float total_angle_y = 0; // pitch
float total_angle_z = 0; // yaw

float mag_heading = 0;       // yaw calculated w/magnetometer
float declination_angle = 2; // depends on location
float rad_to_deg = 180 / 3.141592654f;

/* PID */
const float P_GAIN = 0;
const float I_GAIN = 0;
const float D_GAIN = 0;
const float DESIRED_ANGLE = 0;

float error_x = 0;
float error_y = 0;
float error_z = 0;
float error_alt = 0;

float prev_error_x = 0;
float prev_error_y = 0;
float prev_error_z = 0;
float prev_error_alt = 0;

float pid_x = 0;
float pid_y = 0;
float pid_z = 0;
float pid_alt = 0;

float proportional_x = 0;
float proportional_y = 0;
float proportional_z = 0;
float proportional_alt = 0;

float integral_x = 0;
float integral_y = 0;
float integral_z = 0;
float integral_alt = 0;

float derivative_x = 0;
float derivative_y = 0;
float derivative_z = 0;
float derivative_alt = 0;

void getTotalAngles()
{
  getImu();
  // complementary filters
  float acc_angle_x = atan(-1 * (acc_x / 16384.0) / sqrt(pow((acc_y / 16384.0), 2) + pow((acc_z / 16384.0), 2))) * rad_to_deg;
  float acc_angle_y = atan((acc_y / 16384.0) / sqrt(pow((acc_x / 16384.0), 2) + pow((acc_z / 16384.0), 2))) * rad_to_deg;

  float gyr_angle_x = (gyr_x / 131) * elapsed_time;
  float gyr_angle_y = (gyr_y / 131) * elapsed_time;
  float gyr_angle_z = (gyr_z / 131) * elapsed_time;

  total_angle_x = 0.98 * (total_angle_x + gyr_angle_x) + 0.02 * acc_angle_x;
  total_angle_y = 0.98 * (total_angle_y + gyr_angle_y) + 0.02 * acc_angle_y;
  total_angle_z = gyr_angle_z;

  /*
  float mag_angle_x = total_angle_x;                       // roll
  float mag_angle_y = total_angle_y - (total_angle_y * 2); // mag and gyr axis are opposed
  float mag_angle_z = gyr_angle_z - (gyr_angle_z * 2);

  float mag_x_hor = mag_x * cos(mag_angle_y) + mag_y * sin(mag_angle_x) * sin(mag_angle_y) - mag_z * cos(mag_angle_x) * sin(mag_angle_y);
  float mag_y_hor = mag_y * cos(mag_angle_x) + mag_z * sin(mag_angle_x);
  mag_heading = (atan2(mag_x_hor, mag_y_hor) * rad_to_deg) + declination_angle; */

  Serial.print("  X:");
  Serial.print(total_angle_x, 2);

  Serial.print("  Y:");
  Serial.print(total_angle_y, 2);

  Serial.print("  Z:");
  Serial.print(total_angle_z, 2);
}

void calibrateAngles()
{
}

void initController()
{
  //Configure ESC's
  analogWriteResolution(16);              // 16 bit (0 - 65535)
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

int correct_pitch(int pid_input = 0)
{
  int correction = 0;
  int pitch_forward = abs(pitch);
  int pitch_backward = abs(pitch);

  if (pitch > 1)
  {
    correction = map(pitch_forward, 0, 1023, 0, steer_diff);
    Serial.print("   pitch+: " + (String)(correction + pid_input));
  }

  if (pitch < -1)
  {
    correction = map(pitch_backward, 0, 1023, 0, steer_diff);
    correction = -correction;
    Serial.print("   pitch-: " + (String)(correction + pid_input));
  }

  correction += pid_input;

  left_front -= correction;
  right_front -= correction;
  left_rear += correction;
  right_rear += correction;

  return correction;
}

int correct_roll(int pid_input = 0)
{
  int correction = 0;
  int roll_right = abs(roll);
  int roll_left = abs(roll);

  if (roll > 1)
  {
    correction = map(roll_left, 0, 1023, 0, steer_diff);
    Serial.print("   roll+: " + (String)(correction + pid_input));
  }

  if (roll < -1)
  {
    correction = map(roll_right, 0, 1023, 0, steer_diff);
    correction = -correction;
    Serial.print("   roll-: " + (String)(correction + pid_input));
  }

  correction += pid_input;

  left_side -= correction;
  left_front -= correction * 0.75;
  left_rear -= correction * 0.75;
  right_side += correction;
  right_front += correction * 0.75;
  right_rear += correction * 0.75;

  return correction;
}

int correct_yaw(int pid_input = 0)
{
  int correction = 0;
  int yaw_cw = abs(yaw);
  int yaw_ccw = abs(yaw);

  if (yaw > 1)
  {
    correction = map(yaw_cw, 0, 1023, 0, steer_diff);
    Serial.println("   yaw+: " + (String)(correction + pid_input));
  }

  if (yaw < -1)
  {
    correction = map(yaw_ccw, 0, 1023, 0, steer_diff);
    correction = -correction;
    Serial.println("   yaw-: " + (String)(correction + pid_input));
  }

  correction += pid_input;

  right_front -= correction;
  right_rear -= correction;
  left_side -= correction;
  left_front += correction;
  left_rear += correction;
  right_side += correction;
  return correction;
}

void calculate_pid()
{
  error_x = total_angle_x - DESIRED_ANGLE;
  error_y = total_angle_y - DESIRED_ANGLE;
  error_z = total_angle_z - DESIRED_ANGLE;
  error_alt = relative_altitude - hold_altitude;

  proportional_x = error_x * P_GAIN;
  proportional_y = error_y * P_GAIN;
  proportional_z = error_z * P_GAIN;
  proportional_alt = error_alt * P_GAIN;

  if (abs(error_x) > 0.5 && abs(error_x) < 2) // less than 3 degrees
  {
    integral_x += error_x * I_GAIN;
  }
  else
  {
    integral_x = 0;
  }

  if (abs(error_y) > 0.5 && abs(error_y) < 2)
  {
    integral_y += error_y * I_GAIN;
  }
  else
  {
    integral_y = 0;
  }

  if (abs(error_z) > 0.5 && abs(error_z) < 2)
  {
    integral_z += error_z * I_GAIN;
  }
  else
  {
    integral_z = 0;
  }

  if (error_alt > -0.5 && error_alt < 0.5)
  {
    integral_alt += error_alt * I_GAIN;
  }

  derivative_x = ((error_x - prev_error_x) / elapsed_time) * D_GAIN;
  derivative_y = ((error_y - prev_error_y) / elapsed_time) * D_GAIN;
  derivative_z = ((error_z - prev_error_z) / elapsed_time) * D_GAIN;
  derivative_alt = ((error_alt - prev_error_alt) / elapsed_time) * D_GAIN;

  pid_x = proportional_x + integral_x + derivative_x;
  pid_y = proportional_y + integral_y + derivative_y;
  pid_z = proportional_z + integral_z + derivative_z;
  pid_alt = proportional_alt + integral_alt + derivative_alt;

  //pid_alt = constrain(pid_alt, 0, 4000); // throttle

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
  steer_diff = (throttle - THROTTLE_MIN) / 4;

  left_front = left_side = left_rear = right_front = right_side = right_rear = throttle;

  pitch = (rc_pitch - 127) * 8;
  roll = (rc_roll - 127) * 8;
  yaw = (rc_yaw - 127) * 8;

  //Serial.print((String) "   p:" + pitch + "  r:" + roll);

  if (pitch < 150 && pitch > -100 && roll < 20 && roll > -20)
  {
    calculate_pid();
    Serial.print((String) "   PID:  x:" + pid_x + "  y:" + pid_y);
  }

  //Serial.print("  throttle: " + (String)throttle + "  ");

  correct_pitch(pid_y);
  correct_roll(pid_x);
  correct_yaw(pid_z);

  if (throttle < (THROTTLE_MIN + 50))
  {
    left_front = left_side = left_rear = right_front = right_side = right_rear = 0;
    pid_x = 0;
    pid_y = 0;
    pid_z = 0;
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
