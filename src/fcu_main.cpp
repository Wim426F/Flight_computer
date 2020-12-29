/*  
   Created by:    Wim Boone
   Project:       Drone flight computer
   Date Started:  24/08/2019
 */
#include <EEPROM.h>
#include <TeensyThreads.h>

#include <sensors.h>
#include <communication.h>
#include <flight_controls.h>
#include <globals.h>

#ifdef TARGET_TEENSY35
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the display name (display)

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
#endif

#ifdef TARGET_FCU1062
RF24 radio(7, 8);
ICP101xx icp;
ICM_20948_I2C icm;
#endif

ControlSystem ctrlSys;
RadioCommunication rc;

// hc12 parameters are stored in eeprom
#define CH_STATE_ADRR 0
#define BD_STATE_ADRR 1
#define TP_STATE_ADRR 2
uint8_t channel_state = EEPROM.read(CH_STATE_ADRR);
uint8_t baud_state = EEPROM.read(BD_STATE_ADRR);
uint8_t txpower_state = EEPROM.read(TP_STATE_ADRR);

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

uint16_t rc_throttle = 0;
uint16_t rc_yaw = 0;
uint16_t rc_pitch = 0;
uint16_t rc_roll = 0;
uint8_t rc_button1 = 0;
uint8_t rc_button2 = 0;
uint8_t rc_button3 = 0;
uint8_t rc_button4 = 0;

void thread2()
{
  for (;;)
  {
    ctrlSys.steer();
    threads.yield();
  }
}

void setup()
{
  threads.addThread(thread2);
  threads.start();
  ctrlSys.initialize();
  sensor_status_t sensors_init();
  pwb_status_t pwb_init();

#ifdef TARGET_TEENSY35
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

  gps.begin(115200);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(HC12_COMMAND_MODE, OUTPUT);
  //EEPROM.write(BD_STATE_ADRR, 2);
  HC12.begin((rc_baudrate[baud_state])); // begin at speed last saved in eeprom
  while (!HC12)
    Serial.begin(115200); // Serial port to computer

  digitalWrite(HC12_COMMAND_MODE, HIGH); // don't enter command mode, high
#endif
  pinMode(GIMBAL_DOWN, OUTPUT);
  pinMode(GIMBAL_UP, OUTPUT);
}

void loop()
{
  rc.parseIncomingBytes();

  // 10 ms
  if (since_int1 > interval1)
  {
    since_int1 -= interval1;
  }
  // 100 ms
  if (since_int2 > interval2)
  {
    since_int2 -= interval2;
    Serial.println(" ");
    //Serial.print();
    Serial.print(" ");
    //Serial.print();
    Serial.print(" ");
    //Serial.print();
    Serial.println(" ");
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
