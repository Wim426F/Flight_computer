#include <Arduino.h>
#include <math.h>

#include <flight_controls.h>
#include <globals.h>

const float PWM_FREQ = 50;        //Hz
uint8_t steering_multiplier = 50; // max % power shifted to other side
const uint16_t ESC_MIN_US = 2600; // Microseconds
const uint16_t ESC_MAX_US = 6600; // Microseconds
const uint16_t STEER_RANGE = 2048;

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

float accX = 0;
float accY = 0;
float accZ = 0;
float gyrX = 0;
float gyrY = 0;
float gyrZ = 0;
float magX = 0;
float magY = 0;
float magZ = 0;

float total_angle_x = 0; // roll
float total_angle_y = 0; // pitch

float mag_heading = 0;       // yaw calculated w/magnetometer
float declination_angle = 2; // depends on location
float rad_to_deg = 180 / 3.141592654;

/* PID */
const float CORRECTION_P = 5;
const float CORRECTION_I = 0.005;
const float CORRECTION_D = 3;
const float DESIRED_ANGLE = 0;

float error_x = 0;
float error_y = 0;
float error_alt = 0;
float error_heading = 0;
float prev_error_x = 0;
float prev_error_y = 0;
float prev_error_alt = 0;
float prev_error_heading = 0;
float pid_x = 0;
float pid_y = 0;
float pid_alt = 0;
float pid_heading = 0;
float proportional_x = 0;
float proportional_y = 0;
float proportional_alt = 0;
float proportional_heading = 0;
float integral_x = 0;
float integral_y = 0;
float integral_alt = 0;
float integral_heading = 0;
float derivative_x = 0;
float derivative_y = 0;
float derivative_alt = 0;
float derivative_heading = 0;

void getTotalAngles()
{
  // complementary filters
  float acc_angle_x = atan((accY / 16384.0) / sqrt(pow((accX / 16384.0), 2) + pow((accZ / 16384.0), 2))) * rad_to_deg;
  float acc_angle_y = atan(-1 * (accY / 16384.0) / sqrt(pow((accX / 16384.0), 2) + pow((accZ / 16384.0), 2))) * rad_to_deg;
  float gyr_angle_x = (gyrX / 131) * elapsed_time;
  float gyr_angle_y = (gyrY / 131) * elapsed_time;
  float gyr_angle_z = (gyrZ / 131) * elapsed_time;
  total_angle_x = 0.98 * (total_angle_x + gyr_angle_x) + 0.02 * acc_angle_x;
  total_angle_y = 0.98 * (total_angle_y + gyr_angle_y) + 0.02 * acc_angle_y;

  float mag_angle_x = total_angle_x;                       // roll
  float mag_angle_y = total_angle_y - (total_angle_y * 2); // mag and gyr axis are opposed
  float mag_angle_z = gyr_angle_z - (gyr_angle_z * 2);

  float mag_x_hor = magX * cos(mag_angle_y) + magY * sin(mag_angle_x) * sin(mag_angle_y) - magZ * cos(mag_angle_x) * sin(mag_angle_y);
  float mag_y_hor = magY * cos(mag_angle_x) + magZ * sin(mag_angle_x);
  mag_heading = (atan2(mag_x_hor, mag_y_hor) * rad_to_deg) + declination_angle;
}

void init_controller()
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
}

void calibrate()
{
}

int pitch_correction(int pid_input = 0)
{
  int correction = 0;
  int pitch_forward = abs(pitch);
  int pitch_backward = abs(pitch);

  pitch_forward = constrain(pitch, 0, STEER_RANGE);
  pitch_backward = constrain(pitch, 0, STEER_RANGE);

  if (pitch > 1 || pid_input > 0)
  {
    if (pid_input > 0)
      correction = pid_input;
    if (pitch > 1)
      correction = map(pitch_forward, 0, STEER_RANGE, 0, (40 * steering_multiplier)); // 2000 = 50%
    left_front -= correction;
    right_front -= correction;
    left_rear += correction;
    right_rear += correction;
  }
  if (pitch < -1 && pid_input == 0)
  {
    correction = map(pitch_backward, 0, STEER_RANGE, 0, (40 * steering_multiplier));
    left_front += correction;
    right_front += correction;
    left_rear -= correction;
    right_rear -= correction;
  }
  //Serial.println((String) "pitch+: " + pitch_forward);
  //Serial.println((String) "pitch-: " + pitch_backward);
  return 0;
}

int roll_correction(int pid_input)
{
  int correction = 0;
  int correction_corners = 0;
  int roll_right = abs(roll);
  int roll_left = abs(roll);

  roll_right = constrain(roll, 0, STEER_RANGE);
  roll_left = constrain(roll, 0, STEER_RANGE);

  if (roll > 1 || pid_input > 0)
  {
    if (roll > 1)
    {
      correction = map(roll_right, 0, STEER_RANGE, 0, (40 * steering_multiplier));
      correction_corners = map(roll_right, 0, STEER_RANGE, 0, (30 * steering_multiplier));
    }
    if (pid_input > 0)
    {
      correction = pid_input;
      correction_corners = pid_input * 0.75;
    }
    left_side += correction;
    left_front += correction_corners;
    left_rear += correction_corners;
    right_side -= correction;
    right_front -= correction_corners;
    right_rear -= correction_corners;
  }
  if (roll < -1)
  {
    correction = map(roll_left, 0, STEER_RANGE, 0, (40 * steering_multiplier));
    correction_corners = map(roll_left, 0, STEER_RANGE, 0, (30 * steering_multiplier));
    left_side -= correction;
    left_front -= correction_corners;
    left_rear -= correction_corners;
    right_side += correction;
    right_front += correction_corners;
    right_rear += correction_corners;
  }
  //Serial.println((String) "roll+: " + roll);
  //Serial.println((String) "roll-: " + roll_left);
  return 0;
}

int yaw_correction(int pid_input)
{
  int correction = 0;
  int yaw_cw = abs(yaw);
  int yaw_ccw = abs(yaw);

  if (yaw > 1 || pid_input > 0)
  {
    if (yaw > 1)
    {
      correction = map(yaw_cw, 0, STEER_RANGE, 0, (40 * steering_multiplier));
    }
    if (pid_input > 0)
    {
      correction = pid_input;
    }
    correction = map(yaw_cw, 0, STEER_RANGE, 0, (40 * steering_multiplier));
    right_front += correction;
    right_rear += correction;
    left_side += correction;
    left_front -= correction;
    left_rear -= correction;
    right_side -= correction;
  }
  if (yaw < -1)
  {
    correction = map(yaw_ccw, 0, STEER_RANGE, 0, (40 * steering_multiplier));
    right_front -= correction;
    right_rear -= correction;
    left_side -= correction;
    left_front += correction;
    left_rear += correction;
    right_side += correction;
  }
  return 0;
}

void pid_correction()
{
  getTotalAngles();
  if (hold_position)
  {
    error_x = total_angle_x - DESIRED_ANGLE;
    error_y = total_angle_y - DESIRED_ANGLE;
  }
  else
  {
    error_x = total_angle_x - total_angle_x;
    error_y = total_angle_y - total_angle_y;
  }

  proportional_x = error_x * CORRECTION_P;
  proportional_y = error_y * CORRECTION_P;

  if (error_x > -3 && error_x < 3)
  {
    integral_x += error_x * CORRECTION_I;
  }
  if (error_y > -3 && error_y < 3)
  {
    integral_y += error_y * CORRECTION_I;
  }

  derivative_x = ((error_x - prev_error_x) / elapsed_time) * CORRECTION_D;
  derivative_y = ((error_y - prev_error_y) / elapsed_time) * CORRECTION_D;

  pid_x = proportional_x + integral_x + derivative_x;
  pid_y = proportional_y + integral_y + derivative_y;

  pid_x = constrain(pid_x, 0, STEER_RANGE);
  pid_y = constrain(pid_y, 0, STEER_RANGE);

  prev_error_x = error_x;
  prev_error_y = error_y;
}

void control_loop()
{
  while (true)
  {
    previous_time = current_time;
    current_time = millis();
    elapsed_time = (current_time - previous_time) / 1000;
    uint16_t throttle = map(rc_throttle, 0, (STEER_RANGE * 2), ESC_MIN_US, ESC_MAX_US);
    steering_multiplier /= 4;

    pitch = rc_pitch - STEER_RANGE;
    roll = rc_roll - STEER_RANGE;
    yaw = rc_yaw - STEER_RANGE;

    left_front = throttle;
    left_side = throttle;
    left_rear = throttle;
    right_front = throttle;
    right_side = throttle;
    right_rear = throttle;

    if (pitch && roll && yaw < 10 && hold_position)
    {
      pid_correction();
    }

    if (pitch && roll && yaw < 2)
    {
      pid_correction();
    }

    if ((pitch || roll || yaw > 2) && hold_position)
    {
      hold_position = false;
    }

    pitch_correction(pid_x);
    roll_correction(pid_y);
    yaw_correction(pid_heading);

    left_front = constrain(left_front, ESC_MIN_US, ESC_MAX_US);
    left_side = constrain(left_side, ESC_MIN_US, ESC_MAX_US);
    left_rear = constrain(left_rear, ESC_MIN_US, ESC_MAX_US);
    right_front = constrain(right_front, ESC_MIN_US, ESC_MAX_US);
    right_side = constrain(right_side, ESC_MIN_US, ESC_MAX_US);
    right_rear = constrain(right_rear, ESC_MIN_US, ESC_MAX_US);

    analogWrite(MOTOR1, right_front);
    analogWrite(MOTOR2, left_front);
    analogWrite(MOTOR3, left_side);
    analogWrite(MOTOR4, left_rear);
    analogWrite(MOTOR5, right_rear);
    analogWrite(MOTOR6, right_side);
    /*
    Serial.print(left_front);
    Serial.print(" ");
    Serial.print(left_side);
    Serial.print(" ");
    Serial.print(left_rear);
    Serial.print(" ");
    Serial.print(right_front);
    Serial.print(" ");
    Serial.print(right_side);
    Serial.print(" ");
    Serial.print(right_rear);
    Serial.println(" ");
    */
    threads.yield();
  }
}

void avoid_obstacles()
{
  
}

void enableWaypoint(float longitude, float latitude, float altitude)
{
}
