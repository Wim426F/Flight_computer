/*  
   Created by:    Wim Boone
   Project:       Drone flight computer
   Date Started:  24/08/2019
 */
#include <globals.h>
#include <communication.h>
#include <flight_controls.h>

#ifdef TARGET_TEENSY35
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu6050(Wire);
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

// receiver parameters are stored in eeprom
#define CH_STATE_ADRR 0
#define BD_STATE_ADRR 1
#define TP_STATE_ADRR 2
uint8_t channel_state = EEPROM.read(CH_STATE_ADRR);
uint8_t baud_state = EEPROM.read(BD_STATE_ADRR);
uint8_t txpower_state = EEPROM.read(TP_STATE_ADRR);

#endif

RF24 rf24(7, 8); // NRF24 Radio Transmitter & Receiver
ICP101xx icp; // Barometer
ICM_20948_I2C icm; // 9DOF Motion Sensor
ICM_20948_SPI icm_spi;
RadioCommunication rc;

float relative_altitude = 0;
float abs_airpressure = 0;
float gndlvl_airpressure = 0;
float temperature_icp = 0;
float temperature_icm = 0;
float temperature_avg = 0;

uint16_t rc_throttle = 0;
uint16_t rc_yaw = 0;
uint16_t rc_pitch = 0;
uint16_t rc_roll = 0;
uint8_t rc_button1 = 0;
uint8_t rc_button2 = 0;
uint8_t rc_button3 = 0;
uint8_t rc_button4 = 0;

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

void getScaledAgmt(ICM_20948_AGMT_t agmt);

void setup()
{
  threads.addThread(control_loop);
  threads.start();
  init_controller();
  current_time = millis();
#ifdef TARGET_TEENSY35
  Wire.setClock(400000); // 400kHz
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.println("Hexacopter Drone");
  display.display();

  bmp.begin(0x76);
  // mpu.begin(0x68); 
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(HC12_COMMAND_MODE, OUTPUT);
  //EEPROM.write(BD_STATE_ADRR, 2);
  HC12.begin((rc_baudrate[baud_state])); // begin at speed last saved in eeprom
  Serial.begin(115200);
  digitalWrite(HC12_COMMAND_MODE, HIGH); // don't enter command mode, high
  gndlvl_airpressure = bmp.readPressure() / 100;

  #elif defined(TARGET_FCU1062)
  Wire.setClock(1000000); // 1MHz
  Wire.begin();
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
  
  #ifdef TARGET_TEENSY35
  rc.parseIncomingBytes();
  relative_altitude = bmp.readAltitude(gndlvl_airpressure);
  #endif
  temperature_avg = (temperature_icm + temperature_icp)/2;

  icp.measureStart(icp.NORMAL);
  if(icp.dataReady())
  {
    abs_airpressure = icp.getPressurePa();
    relative_altitude = ((pow((gndlvl_airpressure / abs_airpressure), 1 / 5.257))*(temperature_avg + 273.15))/0.0065;
  } 

  if (icm.dataReady())
  {
    icm.getAGMT();
    getScaledAgmt(icm.agmt);
  }

  if (rc_button1 && rc_button2 == HIGH)
  {
    hold_position = true;
    hold_altitude = relative_altitude;
  } else 
  {
    hold_position = false;
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
  threads.yield(); // if time left in slice was unused go to the next one
}

void getScaledAgmt(ICM_20948_AGMT_t agmt)
{
  accX = icm.accX();
  accY = icm.accY();
  accZ = icm.accZ();

  gyrX = icm.gyrX();
  gyrY = icm.gyrY();
  gyrZ = icm.gyrZ();

  magX = icm.magX();
  magY = icm.magY();
  magZ = icm.magZ();

  temperature_icm = icm.temp();
}