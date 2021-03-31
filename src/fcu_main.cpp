/*  
   Created by:    Wim Boone
   Project:       Drone flight computer
   Date Started:  24/08/2019
 */
#include <globals.h>
#include <hc12.h>
#include <flight_controls.h>
#include <blheli_6way.h>
#include <sensors.h>
#include <util/crc16.h>

#ifdef TARGET_TEENSY35
#include <InternalTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

RF24 rf24(7, 8);   // NRF24 Radio Transmitter & Receiver
ICP101xx icp;      // Barometer
ICM_20948_I2C icm; // 9DOF Motion Sensor
ICM_20948_SPI icm_spi;
HC12 radio;
BlHeli6Way BlHeli;

float relative_altitude = 0;
float abs_airpressure = 0;
float gndlvl_airpressure = 0;
float temperature_icp = 0;
float temperature_icm = 0;
float temperature_avg = 0;

/* Transmitter Variables */
uint16_t rc_throttle = 0;
uint16_t rc_yaw = 0;
uint16_t rc_pitch = 0;
uint16_t rc_roll = 0;
uint8_t rc_param1 = 0;
uint8_t rc_param2 = 0;
uint8_t rc_param3 = 0;
uint8_t rc_param4 = 0;

const long interval1 = 1;
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
bool fcu_configmode = false;

uint16_t crc_xmodem(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    crc = crc_xmodem_update(crc, data[i]);
  }
  return crc;
}

void getScaledAgmt(ICM_20948_AGMT_t agmt)
{
  acc_x = icm.accX();
  acc_y = icm.accY();
  acc_z = icm.accZ();

  gyr_x = icm.gyrX();
  gyr_y = icm.gyrY();
  gyr_z = icm.gyrZ();

  mag_x = icm.magX();
  mag_y = icm.magY();
  mag_z = icm.magZ();

  temperature_icm = icm.temp();
}

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

  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X16, /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16    /* Filtering. */
  );

  mpu.initialize();
  mpu.setXAccelOffset(-2024);
  mpu.setYAccelOffset(-1719);
  mpu.setZAccelOffset(1843);
  mpu.setXGyroOffset(9);
  mpu.setYGyroOffset(31);
  mpu.setZGyroOffset(33);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(HC12_CMD_MODE, OUTPUT);

  Serial.begin(SERIAL_BAUDRATE);
  //delay(2000);
  hc12_uart.begin(rc_baudrate[last_baud]); // begin at speed last saved in eeprom

  //Serial.println((String) "hc12_uart started at baud: " + rc_baudrate[last_baud] + "\n");
  //Serial.print("Setting baudrate: ");
  //radio.setBaudRate(0); // 3 = 9600bps
  //Serial.print("Setting channel: ");
  //radio.setChannel(1);
  //Serial.print("Setting power: ");
  //radio.setTxPower(8);
  /*
  Serial.print("Setting Mode: ");
  radio.setTransmitMode(3); */
  radio.getVersion();
  gndlvl_airpressure = bmp.readPressure();
  initController();

#elif defined(TARGET_FCU1062)

  Wire.setClock(1000000); // 1MHz
  Wire1.setClock(400000);
  Wire.begin();
  Wire1.begin();

  /* Pressure Sensor  */
  icp.begin();
  //FAST: ~3ms, NORMAL: ~7ms (default), ACCURATE: ~24ms, VERY_ACCURATE: ~95ms
  icp.measureStart(icp.VERY_ACCURATE);
  while (!icp.dataReady())
  {
    static uint64_t loopstart = millis();
    if (millis() - loopstart > 200)
    {
      Serial.println("Pressure sensor not found");
      break;
    }
  }
  if (icp.dataReady())
  {
    gndlvl_airpressure = icp.getPressurePa() / 100;
  }

  /*  Motion Sensor  */
  icm.begin();
  icm.swReset();
  if (icm.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("Software Reset returned: "));
    Serial.println(icm.statusString());
  }
  delay(250);
  icm.sleep(false);
  icm.lowPower(false);
  // Sample configuration
  icm.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (icm.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setSampleMode returned: "));
    Serial.println(icm.statusString());
  }
  // Scale configuration
  ICM_20948_fss_t f_scale_settings;
  f_scale_settings.a = gpm2;    // 2G acceleration scale
  f_scale_settings.g = dps2000; // 2000 degrees/second gyro scale
  icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), f_scale_settings);
  if (icm.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setFullScale returned: "));
    Serial.println(icm.statusString());
  }
  // Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t low_pass_filter;
  low_pass_filter.a = acc_d473bw_n499bw;
  low_pass_filter.g = gyr_d361bw4_n376bw5;
  icm.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), low_pass_filter);
  if (icm.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setDLPcfg returned: "));
    Serial.println(icm.statusString());
  }

#endif
}

void loop()
{
  if (Serial.available() > 0) // Serial commands
  {
    delay(50);
    Serial.println("command mode");
    char param = Serial.read();
    if (param == 'b')
    {
      int i = Serial.parseInt();
      radio.setBaudRate(i);
      delay(100);
    }

    if (param == 'c')
    {
      int i = Serial.parseInt();
      radio.setChannel(i);
      delay(100);
    }

    if (param == 'm')
    {
      int i = Serial.parseInt();
      radio.setTransmitMode(i);
      delay(100);
    }

    if (param == 'p')
    {
      int i = Serial.parseInt();
      radio.setTxPower(i);
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

    if (param == 'h')
    {
      int i = Serial.parseInt();
      if (i == 0)
      {
        Serial.println("Config Mode disabled!");
        fcu_configmode = false;
      }

      if (i == 1)
      {
        Serial.println("Config Mode enabled!");
        fcu_configmode = true;
      }

      delay(100);
    }

    if (param == 'd')
    {
      radio.setAllDefault();
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

    while (hc12_uart.available() && hc12_uart.read()); // empty buffer again
    while (Serial.available() && Serial.read()); // empty buffer again
  }

  int button1_state = digitalRead(BUTTON1);
  int button2_state = digitalRead(BUTTON2);

  if (button1_state == LOW || button2_state == LOW)
  {
    //mpu.calcGyroOffsets(true);
    calibrateAngles();
    delay(100);
  }

  if (button1_state == LOW)
  {
    //calibrateAngles();
  }

  if (button2_state == LOW)
  {
  }

  /* Configuration Modus */
  if (button1_state == LOW && button2_state == LOW && fcu_configmode == false)
  {
    Serial.println("Config Mode enabled!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Config Mode");
    fcu_configmode = true;
  }

  if ((button2_state == LOW && fcu_configmode == true))
  {
    Serial.println("Config Mode disabled!");
    fcu_configmode = false;
  }

  if (fcu_configmode == true)
  {
    BlHeli.HostInterface();
  }
  else
  {
    time = micros();
    long int loop_time = 1000000 / (time - prev_time);
    prev_time = time;

#ifdef TARGET_TEENSY35
    radio.readData();
    

#elif defined(TARGET_FCU1062)

    /* Barometer */
    icp.measureStart(icp.NORMAL);
    if (icp.dataReady())
    {
      abs_airpressure = icp.getPressurePa();
      temperature_icp = icp.getTemperatureC();
    }

    /* IMU */
    if (icm.dataReady())
    {
      icm.getAGMT();
      getScaledAgmt(icm.agmt);
    }
#endif

    // 1 ms
    if (since_int1 > interval1)
    {
      since_int1 -= interval1;
    }
    // 100 ms
    if (since_int2 > interval2)
    { /*
      Serial.print("  ox: ");
      Serial.print(offset_gyr_x);
      Serial.print("  oy: ");
      Serial.print(offset_gyr_y);
      Serial.print("  oz: ");
      Serial.println(offset_gyr_z); */
      /*
      Serial.print("  tota-off: ");
      Serial.print(total_angle_x - offset_gyr_x);
      

      Serial.print((String) "  Cycles/s: " + loop_time);
      Serial.print("  CPU temp: ");
      Serial.println(InternalTemperature.readTemperatureC(), 2); */
      
      mainControl();
      
      since_int2 -= interval2;
    }

    // 500 ms
    if (since_int3 > interval3)
    {
      /* Barometer */
      temperature_icm = mpu.getTemperature();
      abs_airpressure = bmp.readPressure();
      temperature_icp = bmp.readTemperature();
      relative_altitude = ((pow((gndlvl_airpressure / abs_airpressure), (1 / 5.257)) - 1) * (temperature_avg + 273.15)) / 0.0065;
      temperature_avg = (temperature_icm + temperature_icp) / 2;
      since_int3 -= interval3;
    }
    // 1000 ms
    if (since_int4 > interval4)
    {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(relative_altitude);
      display.print("m");
      display.display();

      since_int4 -= interval4;
    }
    // 5000 ms
    if (since_int5 > interval5)
    {
      //powerMonitor();
      since_int5 -= interval5;
    }
  }
}