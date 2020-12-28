#include <Arduino.h>
#include <math.h>

#include <sensors.h>
#include <flight_controls.h>
#include <globals.h>

float freq = 50; //Hz

uint16_t left_front = 0;  // M2: CW
uint16_t left_side = 0;   // M3: CCW
uint16_t left_rear = 0;   //M4: CW
uint16_t right_front = 0; // M1: CCW
uint16_t right_side = 0;  // M6: CW
uint16_t right_rear = 0;  // M5: CCW

uint8_t steering_multiplier = 50;  // max % power shifted to other side 

void ControlSystem::initialize()
{
  //Configure ESC's
  analogWriteResolution(16);            // 16 bit (0 - 65535)
  analogWriteFrequency(MOTOR1, freq); // FTM0 timer, pin 5, 6
  analogWriteFrequency(MOTOR5, freq); // FTM1 timer, pin 4
  analogWriteFrequency(MOTOR2, freq); // FTM3 timer, pin 7, 8 ,14

  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT);
  pinMode(MOTOR4, OUTPUT);
  pinMode(MOTOR5, OUTPUT);
  pinMode(MOTOR6, OUTPUT);
  return;
}

void ControlSystem::calibrate()
{
}

void ControlSystem::steer()
{
  uint16_t throttle = map(rc_throttle, 0, 255, 2600, 6600);
  steering_multiplier /= 4;

  left_front = throttle;
  left_side = throttle;
  left_rear = throttle;
  right_front = throttle;
  right_side = throttle;
  right_rear = throttle;

  pitch_correction();
  roll_correction();
  yaw_correction();
/*
  Serial.println(left_front);
  Serial.println(left_side);
  Serial.println(left_rear);

  Serial.println(right_front);
  Serial.println(right_side);
  Serial.println(right_rear);
  Serial.println(" ");
*/
  analogWrite(MOTOR1, right_front);
  analogWrite(MOTOR2, left_front);
  analogWrite(MOTOR3, left_side);
  analogWrite(MOTOR4, left_rear);
  analogWrite(MOTOR5, right_rear);
  analogWrite(MOTOR6, right_side);
}

int ControlSystem::pitch_correction()
{
  int correction = 0;
  int pitch = (int)rc_pitch - 130;
  int pitch_forward = 0;
  int pitch_backwards = 0;

  pitch_forward = constrain(rc_pitch, 0, 129);
  pitch_backwards = constrain(rc_pitch, 0, 129);
  pitch_forward = abs(pitch);
  pitch_backwards = abs(pitch);

  if (pitch != 1)
  {
    if (pitch > 1)
    {
      correction = map(pitch_forward, 0, 129, 0, (40 * steering_multiplier)); // 400 = 10%
      left_front -= correction;
      right_front -= correction;
      left_rear += correction;
      right_rear += correction;
    }
    if (pitch < 1)
    {
      correction = map(pitch_backwards, 0, 129, 0, (40 * steering_multiplier)); // 400 = 10%
      
      left_front += correction;
      right_front += correction;
      left_rear -= correction;
      right_rear -= correction;
    }
    //Serial.println((String) "pitch+: " + pitch_forward);
    //Serial.println((String) "pitch-: " + pitch_backwards);
  }

  return 0;
}

int ControlSystem::roll_correction()
{
  int correction = 0;
  int correction_corners = 0;
  int roll = (int)rc_roll - 128;
  int roll_right = 0;
  int roll_left = 0;

  roll_right = constrain(roll, 127, 255);
  roll_left = constrain(roll, 0, 127);
  roll_right = abs(roll);
  roll_left = abs(roll);

  if (roll != 1)
  {
    if (roll > 1)
    {
      correction = map(roll_right, 0, 128, 0, (40 * steering_multiplier));
      correction_corners = map(roll_right, 0, 128, 0, (30 * steering_multiplier));
      left_side += correction;
      left_front += correction_corners;
      left_rear += correction_corners;
      right_side -= correction;
      right_front -= correction_corners;
      right_rear -= correction_corners;
    }
    if (roll < 1)
    {
      correction = map(roll_left, 0, 128, 0, (40 * steering_multiplier));
      correction_corners = map(roll_left, 0, 128, 0, (30 * steering_multiplier));
      left_side -= correction;
      left_front -= correction_corners;
      left_rear -= correction_corners;
      right_side += correction;
      right_front += correction_corners;
      right_rear += correction_corners;
    }
    //Serial.println((String) "roll+: " + roll);
    //Serial.println((String) "roll-: " + roll_left);
  }

  return 0;
}

int ControlSystem::yaw_correction()
{

  return 0;
}

// Compute steering input with PID
void ControlSystem::hover(float alt)
{
}

void ControlSystem::enableWaypoint(float longitude, float latitude, float altitude)
{
}
