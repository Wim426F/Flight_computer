#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
//#include <SD.h>

#ifdef TARGET_FCU1062
#include <ICM_20948.h>
#include <icp101xx.h>
#include <RF24.h>
#include <SparkFun_Ublox_Arduino_Library.h>

extern ICM_20948_I2C icm;
extern ICP101xx icp;
extern RF24 radio;

// GPIO
#define RF24_IRQ 42
#define RF24_CSN 43
#define IMU_INTR 17
#define LED_RED 11
#define GIMBAL_DOWN 29
#define GIMBAL_UP 30
#define MOTOR1 6
#define MOTOR2 14
#define MOTOR3 7
#define MOTOR4 8
#define MOTOR5 4
#define MOTOR6 5

#endif

#ifdef TARGET_TEENSY35
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>

#define HC12 Serial4
#define gps Serial5
// GPIO
#define HC12_COMMAND_MODE 39
#define BUTTON1 33
#define BUTTON2 34
#define PWM_PIN 3
#define BUZZER 2
#define IMU_INTR 17
#define LED_RED 11
#define LED_GREEN 12
#define GIMBAL_DOWN 29
#define GIMBAL_UP 30
#define SONAR_TRIGGER 22
#define SONAR_ECHO 10
#define MOTOR1 6
#define MOTOR2 14
#define MOTOR3 7
#define MOTOR4 8
#define MOTOR5 4
#define MOTOR6 5

extern Adafruit_MPU6050 mpu;
extern Adafruit_BMP280 bmp;

extern float bmp_pressure;
extern float bmp_altitude;
#endif

extern float groundlvl_pressure;
extern uint8_t steering_multiplier;
extern const float rc_txpower[];
extern const uint32_t rc_baudrate[];

extern uint16_t rc_throttle;
extern uint16_t rc_yaw;
extern uint16_t rc_pitch;
extern uint16_t rc_roll;
extern uint8_t rc_button1;
extern uint8_t rc_button2;
extern uint8_t rc_button3;
extern uint8_t rc_button4;


#endif