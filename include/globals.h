#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
//#include <SD.h>
#include <Wire.h>
#include <TeensyThreads.h>
#include <EEPROM.h>
#include <ICM_20948.h>
#include <icp101xx.h>
#include <RF24.h>
#include <SparkFun_Ublox_Arduino_Library.h>

/* EEPROM Adresses */
#define CH_STATE_ADDR 0
#define BD_STATE_ADDR 1
#define TP_STATE_ADDR 2
#define OFFSET_X_ADDR 10
#define OFFSET_Y_ADDR 20
#define OFFSET_Z_ADDR 30

extern long SERIAL_BAUDRATE;

#ifdef TARGET_TEENSY40
#define MOTOR1 6
#define MOTOR2 14
#define MOTOR3 7
#define MOTOR4 8
#define MOTOR5 4
#define MOTOR6 5
#endif

#ifdef TARGET_FCU1062

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
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>

extern Adafruit_BMP280 bmp;
extern MPU6050 mpu6050;

#define HC12 Serial4
#define gps Serial5

// GPIO
#define HC12_COMMAND_MODE 39
#define BUTTON1 35
#define BUTTON2 36
#define PWM_PIN 3
#define BUZZER 2
#define IMU_INTR 17
#define LED_RED 11
#define LED_GREEN 12
#define GIMBAL_DOWN 29
#define GIMBAL_UP 30
#define MOTOR1 6
#define MOTOR2 14
#define MOTOR3 7
#define MOTOR4 8
#define MOTOR5 4
#define MOTOR6 5
#endif

extern double elapsed_time, current_time, previous_time;
extern float groundlvl_pressure;
extern float abs_airpressure;
extern float relative_altitude;

/* Transmitter Variables */
extern uint16_t rc_throttle;
extern uint16_t rc_yaw;
extern uint16_t rc_pitch;
extern uint16_t rc_roll;
extern uint8_t rc_button1;
extern uint8_t rc_button2;
extern uint8_t rc_button3;
extern uint8_t rc_button4;

extern uint16_t left_front;  // M2: CW
extern uint16_t left_side;   // M3: CCW
extern uint16_t left_rear;   // M4: CW
extern uint16_t right_front; // M1: CCW
extern uint16_t right_side;  // M6: CW
extern uint16_t right_rear;  // M5: CCW

extern float total_angle_x;
extern float total_angle_y;
extern float total_angle_z;

extern float acc_x, acc_y, acc_z;
extern float gyr_x, gyr_y, gyr_z;
extern float mag_x, mag_y, mag_z;

#endif