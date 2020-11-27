#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>

#define HC12 Serial4
extern MPU6050 mpu6050;
extern SoftwareSerial gpsSerial; 
extern Adafruit_BMP280 bmp;

extern float bmp_pressure;
extern float groundlvl_pressure;
extern float bmp_altitude;

extern uint8_t correction_multiplier;
extern const float rc_txpower[];
extern const uint32_t rc_baudrate[];

extern const uint8_t hc_set;
extern const uint8_t button1;
extern const uint8_t button2;
extern const uint8_t servo;
extern const uint8_t buzzer;
extern const uint8_t led_red;
extern const uint8_t led_green;
extern const uint8_t gimbal_down;
extern const uint8_t gimbal_up;
extern const uint8_t sonar_trigg_pin;
extern const uint8_t sonar_echo_pin;

//ESC's
extern const uint8_t esc1_pin;
extern const uint8_t esc2_pin;
extern const uint8_t esc3_pin;
extern const uint8_t esc4_pin;
extern const uint8_t esc5_pin;
extern const uint8_t esc6_pin;

extern uint8_t rc_throttle;
extern uint8_t rc_yaw;
extern uint8_t rc_pitch;
extern uint8_t rc_roll;
extern uint8_t rc_button1;
extern uint8_t rc_button2;
extern uint8_t rc_button3;
extern uint8_t rc_button4;

#endif