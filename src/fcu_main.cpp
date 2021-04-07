/*  
   Created by:    Wim Boone
   Project:       Drone flight computer
   Date Started:  24/08/2019
 */
#include <globals.h>
#include <hc12.h>
#include <flight_controller.h>
#include <blheli_6way.h>
#include <sensors.h>
#include <pw_monitor.h>
#include <util/crc16.h>

#ifdef TARGET_TEENSY35
#include <InternalTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

RF24 rf24(7, 8); // NRF24 Radio Transmitter & Receiver
HC12 hc12;
BlHeli6Way BlHeli;

/* Transmitter Variables */
uint16_t rc_throttle = 0;
uint16_t rc_yaw = 0;
uint16_t rc_pitch = 0;
uint16_t rc_roll = 0;
uint8_t rc_param1 = 0;
uint8_t rc_param2 = 0;
uint8_t rc_param3 = 0;
uint8_t rc_param4 = 0;

const long interval1 = 4;
const long interval2 = 10;
const long interval3 = 500;
const long interval4 = 1000;
const long interval5 = 5000;
elapsedMillis since_int1;
elapsedMillis since_int2;
elapsedMillis since_int3;
elapsedMillis since_int4;
elapsedMillis since_int5;

long int time = 0;
long int prev_time = 0;

long SERIAL_BAUDRATE = 2000000;

uint16_t crc_xmodem(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    crc = crc_xmodem_update(crc, data[i]);
  }
  return crc;
}

#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

void _softRestart()
{
  Serial.end();           //clears the serial monitor  if used
  SCB_AIRCR = 0x05FA0004; //write value for restart
}

void setup()
{
  current_time = millis();

#ifdef TARGET_TEENSY35
  Wire.setClock(400000); // 400kHz
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.clearDisplay();
  display.println("Hexacopter Drone");
  display.display();

  InternalTemperature.begin(TEMPERATURE_NO_ADC_SETTING_CHANGES);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(HC12_CMD_MODE, OUTPUT);

  Serial.begin(SERIAL_BAUDRATE);
  hc12_uart.begin(rc_baudrate[last_baud]); // begin at speed last saved in eeprom

  //Serial.println((String) "hc12_uart started at baud: " + rc_baudrate[last_baud] + "\n");
  //Serial.print("Setting baudrate: ");
  //radio.setBaudRate(0); // 3 = 9600bps
  ///Serial.print("Setting Mode: ");
  //radio.setTransmitMode(3);
  //radio.getVersion();

  initAllSensors();
  initController();

#elif defined(TARGET_FCU1062)
  Wire.setClock(1000000); // 1MHz
  Wire1.setClock(400000);
  Wire.begin();
  Wire1.begin();
#endif
}

void loop()
{
  if (Serial.available() > 0) // Serial commands
  {
    delay(50);
    Serial.println("command mode");
    char param = Serial.read();

    if (param == 'p')
    {
      float i = Serial.parseFloat();
      p_gain = i;
      Serial.println((String) "gain p" + i);
      delay(2000);
    }

    if (param == 'i')
    {
      float i = Serial.parseFloat();
      i_gain = i;
      Serial.println((String) "gain i" + i);
      delay(2000);
    }

    if (param == 'd')
    {
      float i = Serial.parseFloat();
      d_gain = i;
      Serial.println((String) "gain d" + i);
      delay(2000);
    }

    if (param == 'e')
    {
      float i = Serial.parseFloat();
      angle_x_setpoint = i;
      Serial.println((String) "gain d" + i);
      delay(2000);
    }

    if (param == 'h')
    {
      Serial.println("Config Mode enabled!");
      BlHeli.HostInterface();
    }

#ifdef TARGET_TEENSY35
    if (param == 'b')
    {
      int i = Serial.parseInt();
      hc12.setBaudRate(i);
      delay(100);
    }

    if (param == 'c')
    {
      int i = Serial.parseInt();
      hc12.setChannel(i);
      delay(100);
    }

    if (param == 'm')
    {
      int i = Serial.parseInt();
      hc12.setTransmitMode(i);
      delay(100);
    }

    if (param == 's')
    {

      int i = Serial.parseInt();
      Serial.print("HC12 started at: ");
      EEPROM.write(BD_STATE_ADDR, i);
      last_baud = EEPROM.read(BD_STATE_ADDR);

      Serial.println(rc_baudrate[last_baud]);
      hc12_uart.end();
      hc12_uart.begin(rc_baudrate[last_baud]);
      delay(100);
    }

    if (param == 'q')
    {
      hc12.setAllDefault();
      delay(100);
    }

    if (param == 'r')
    {
      Serial.println("Rebooting...");
      delay(500);
      _softRestart();
    }

    if (param == 'a')
    {
      String param = Serial.readString();
      digitalWrite(HC12_CMD_MODE, LOW);
      delay(40);
      hc12_uart.print(param);
      delay(100);
      digitalWrite(HC12_CMD_MODE, HIGH);
      Serial.println(hc12_uart.readString());
      delay(1000);
    }
    while (hc12_uart.available() && hc12_uart.read())
      ; // empty buffer again
#endif

    while (Serial.available() && Serial.read())
      ; // empty buffer again
  }

  int button1_state = digitalRead(BUTTON1);
  int button2_state = digitalRead(BUTTON2);

  if (button1_state == LOW)
  {
  }

  if (button2_state == LOW)
  {
  }

  /* Configuration Modus */
  if (button1_state == LOW && button2_state == LOW)
  {
    Serial.println("Config Mode enabled!");
#ifdef TARGET_TEENSY35
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Config Mode");
#endif
    BlHeli.HostInterface(); // this is a loop
  }

  time = micros();
  long int loop_time = 1000000 / (time - prev_time);
  prev_time = time;

#ifdef TARGET_TEENSY35

  hc12.readData();

#endif

  // 1 ms
  if (since_int1 > interval1)
  {
    mainControl();
    since_int1 -= interval1;
  }
  // 100 ms
  if (since_int2 > interval2)
  {
    /*
      Serial.print("  tota-off: ");
      Serial.print(total_angle_x - offset_gyr_x);
      

      Serial.print((String) "  Cycles/s: " + loop_time);
      Serial.print("  CPU temp: ");
      Serial.println(InternalTemperature.readTemperatureC(), 2); */

    since_int2 -= interval2;
  }

  // 500 ms
  if (since_int3 > interval3)
  {
#ifdef TARGET_TEENSY35
    updateBarometer();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(relative_altitude);
    display.print("m");
    display.display();
#endif

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
    powerMonitor();
    since_int5 -= interval5;
  }
}