/*  
   Created by:    Wim Boone
   Project:       Drone flight computer
   Date Started:  24/08/2019
 */
#include <EEPROM.h>
#include <TeensyThreads.h> 
#include "SD_t3.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <sensors.h>
#include <communication.h>
#include <flight_controls.h>
#include <globals.h>

// OLED display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the display name (display)
MPU6050 mpu6050(Wire);
SoftwareSerial gpsSerial(35, 36); // (rx,tx)
Adafruit_BMP280 bmp;
ControlSystem ctrlSys;
RadioCommunication rc;

// hc12 parameters are stored in eeprom
const uint8_t ch_state_adrr = 0;
const uint8_t bd_state_adrr = 1;
const uint8_t tp_state_adrr = 2;
uint8_t channel_state = EEPROM.read(ch_state_adrr);
uint8_t baud_state = EEPROM.read(bd_state_adrr);
uint8_t txpower_state = EEPROM.read(tp_state_adrr);

const long interval1 = 10;
const long interval2 = 100;
const long interval3 = 500;
const long interval4 = 1000;
const long interval5 = 5000;
elapsedMillis since_int1;
elapsedMillis since_int2;
elapsedMillis since_int3;
elapsedMillis since_int4;
elapsedMillis since_int5;

float groundlvl_pressure = 0;
float bmp_pressure = 0;
float bmp_altitude = 0;

const uint8_t hc_set = 39;
const uint8_t button1 = 33;
const uint8_t button2 = 34;
const uint8_t servo = 3;
const uint8_t buzzer = 2;
const uint8_t led_red = 11;
const uint8_t led_green = 12;
const uint8_t gimbal_down = 29;
const uint8_t gimbal_up = 30;
const uint8_t sonar_trigg_pin = 22;
const uint8_t sonar_echo_pin = 10;

//ESC's
const uint8_t esc1_pin = 6;
const uint8_t esc2_pin = 14;
const uint8_t esc3_pin = 7;
const uint8_t esc4_pin = 8;
const uint8_t esc5_pin = 4;
const uint8_t esc6_pin = 5;

uint8_t rc_throttle = 0;
uint8_t rc_yaw = 0;
uint8_t rc_pitch = 0;
uint8_t rc_roll = 0;
uint8_t rc_button1 = 0;
uint8_t rc_button2 = 0;
uint8_t rc_button3 = 0;
uint8_t rc_button4 = 0;


void thread2()
{
  for(;;)
  {
    ctrlSys.steer();
    threads.yield();
  }
}

void setup()
{
  ctrlSys.initialize();
  //rc.config(1, 4800);
  sensor_status_t sensors_init();
  pwb_status_t pwb_init();
  
  // OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.print("Wim Boone");
  display.setCursor(15, 20);
  display.setTextSize(1);
  display.println("Hexacopter Drone");
  display.display();
  delay(1000);

  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  
  pinMode(buzzer, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(gimbal_down, OUTPUT); // Camera
  pinMode(gimbal_up, OUTPUT);
  pinMode(hc_set, OUTPUT);
  digitalWrite(hc_set, HIGH);

  HC12.begin(2400);
  //HC12.begin((rc_baudrate[baud_state])); // begin at speed last saved in eeprom
  gpsSerial.begin(115200); 
  Serial.begin(115200); // Serial port to computer
  threads.addThread(thread2);
  threads.start();
  //threads.setTimeSlice(thread_id, 1); // for thread2 
  //EEPROM.write(bd_state_adrr, 2);
  rc.setBaudRate(4800);
  rc.setChannel(1);
} 

void loop()
{
  if (HC12.available() > 0)
  {
    rc.receive();
  }
  // 10 ms
  if (since_int1 > interval1)
  {
    since_int1 -= interval1;
    
  }
  // 100 ms
  if (since_int2 > interval2)
  {
    since_int2 -= interval2;
    
  }
  
  // 500 ms
  if (since_int3 > interval3)
  {
    since_int3 -= interval3;
    
  }
  // 1000 ms
  if (since_int4 > interval4)
  {
    since_int4 -= interval4;
    
  }
  // 5000 ms
  if (since_int5 > interval5)
  {
    since_int5 -= interval5;
    
  }

}

