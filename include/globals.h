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

#define OFS_GYR_X_ADDR 10
#define OFS_GYR_Y_ADDR 15
#define OFS_GYR_Z_ADDR 20

#define OFS_ACC_X_ADDR 25
#define OFS_ACC_Y_ADDR 30
#define OFS_ACC_Z_ADDR 35

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
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <hc12.h>

extern Adafruit_BMP280 bmp;
extern MPU6050 mpu;
extern HC12 radio;

#define hc12_uart Serial4
#define gps Serial5

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
extern float groundlvl_pressure;
extern float abs_airpressure;
extern float relative_altitude;

extern float temperature_avg;
extern float temperature_icp;
extern float temperature_icm;

/* Transmitter Variables */
extern uint16_t rc_throttle;
extern uint16_t rc_yaw;
extern uint16_t rc_pitch;
extern uint16_t rc_roll;
extern uint8_t rc_param1;
extern uint8_t rc_param2;
extern uint8_t rc_param3;
extern uint8_t rc_param4;

extern uint16_t crc_xmodem(const uint8_t *data, uint16_t len);

#endif