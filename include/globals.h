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
#define TM_STATE_ADDR 3

extern long SERIAL_BAUDRATE;

#define gps_uart Serial5

#ifdef TARGET_FCU1062

extern ICM_20948_I2C icm;
extern ICP101xx icp;
extern RF24 rf24;

// GPIO
#define BUTTON1 35
#define BUTTON2 36
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
#include <Adafruit_BMP280.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <hc12.h>

extern Adafruit_BMP280 bmp;
extern MPU6050 mpu;
extern HC12 hc12;

#define hc12_uart Serial4

// GPIO
#define HC12_CMD_MODE 39
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

/* Transmitter Variables */
extern byte rc_throttle;
extern byte rc_yaw;
extern byte rc_pitch;
extern byte rc_roll;
extern byte rc_param1;
extern byte rc_param2;
extern byte rc_param3;
extern byte rc_param4;

extern uint16_t crc_xmodem(const uint8_t *data, uint16_t len);

#endif