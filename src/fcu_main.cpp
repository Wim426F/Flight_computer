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
#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location
#define PWB_ADDR_IIC 1                               // Powerboard address
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu6050(Wire);
Adafruit_BMP280 bmp;
#define PWB_ADDR_IIC 1

// receiver parameters are stored in eeprom
uint8_t channel_state = EEPROM.read(CH_STATE_ADDR);
uint8_t baud_state = EEPROM.read(BD_STATE_ADDR);
uint8_t txpower_state = EEPROM.read(TP_STATE_ADDR);

#endif

RF24 rf24(7, 8);   // NRF24 Radio Transmitter & Receiver
ICP101xx icp;      // Barometer
ICM_20948_I2C icm; // 9DOF Motion Sensor
ICM_20948_SPI icm_spi;
RadioCommunication rc;

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
uint8_t rc_button1 = 0;
uint8_t rc_button2 = 0;
uint8_t rc_button3 = 0;
uint8_t rc_button4 = 0;

/* Powerboard Variables */
byte pwb_data[21];
float motor1_amps = 0;
float motor2_amps = 0;
float battery_amps = 0;
float battery_temp = 0;
float battery_voltage = 0;
byte pwb_status = 0;

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

void fetchPwb();

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

  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X16, /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16    /* Filtering. */
  );
  mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(HC12_COMMAND_MODE, OUTPUT);

  HC12.begin((rc_baudrate[baud_state])); // begin at speed last saved in eeprom
  Serial.begin(115200);
  digitalWrite(HC12_COMMAND_MODE, HIGH); // don't enter command mode, high
  gndlvl_airpressure = bmp.readPressure();
  initController();
#elif defined(TARGET_FCU1062)

  Wire.setClock(1000000); // 1MHz
  Wire1.setClock(400000);
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
  if (Serial.available())
  {
    String input;
    if (isDigit(Serial.peek()))
      rc_throttle = Serial.parseInt();
    else
      input = Serial.readString();

    if (input == "reboot")
      _softRestart();
    if (input == "calibrate")
      calibrateAngles();
  }

  if (digitalRead(BUTTON1) == LOW)
    calibrateAngles();
  if (digitalRead(BUTTON2) == LOW)
    _softRestart();
#ifdef TARGET_TEENSY35
  //rc.parseIncomingBytes();

  /* Barometer */
  abs_airpressure = bmp.readPressure();
  temperature_icp = bmp.readTemperature();

  /* IMU */
  mpu6050.update();

  acc_x = mpu6050.getRawAccX();
  acc_y = mpu6050.getRawAccY();
  acc_z = mpu6050.getRawAccZ();
  gyr_x = mpu6050.getRawGyroX();
  gyr_y = mpu6050.getRawGyroY();
  gyr_z = mpu6050.getRawGyroY();

  temperature_icm = mpu6050.getTemp();

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

  relative_altitude = ((pow((gndlvl_airpressure / abs_airpressure), (1 / 5.257)) - 1) * (temperature_avg + 273.15)) / 0.0065;
  temperature_avg = (temperature_icm + temperature_icp) / 2;

  mainControl();

  // 10 ms
  if (since_int1 > interval1)
  {
    since_int1 -= interval1;
  }
  // 100 ms
  if (since_int2 > interval2)
  { /*
    Serial.print(left_front);
    Serial.print(" lf ");

    Serial.print(left_side);
    Serial.print(" ls ");

    Serial.print(left_rear);
    Serial.print(" lr ");

    Serial.print(right_front);
    Serial.print(" rf ");

    Serial.print(right_side);
    Serial.print(" rs ");

    Serial.print(right_rear);
    Serial.println(" rr "); 
    Serial.print(total_angle_x);
    Serial.print(" X:");
    Serial.print(total_angle_y);
    Serial.println(" Y:"); */
    Serial.println((String) "    X: " + total_angle_x + "  Y: " + total_angle_y);
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
    fetchPwb();
    since_int5 -= interval5;
  }
}

void fetchPwb()
{
  Wire.beginTransmission(PWB_ADDR_IIC);
  Wire.requestFrom(PWB_ADDR_IIC, 24);

  if (Wire.available())
  {
    int i = 0;
    while (Wire.available()) // slave may send less than requested
    {
      pwb_data[i] = Wire.receive(); // receive a byte as character
      i = i + 1;
    }

    //A union datatypes makes the byte and float elements share the same piece of memory, which enables conversion from a byte array to a float possible
    union Param1
    {
      byte bytes[4];
      float fvalue;
    } current_tot_u;
    current_tot_u.bytes[0] = pwb_data[0];
    current_tot_u.bytes[1] = pwb_data[1];
    current_tot_u.bytes[2] = pwb_data[2];
    current_tot_u.bytes[3] = pwb_data[3];
    battery_amps = current_tot_u.fvalue * 60;

    union Param2
    {
      byte bytes[4];
      float fvalue;
    } DRPS_Union; //DRPS = Drum Revs per Second
    DRPS_Union.bytes[0] = pwb_data[4];
    DRPS_Union.bytes[1] = pwb_data[5];
    DRPS_Union.bytes[2] = pwb_data[6];
    DRPS_Union.bytes[3] = pwb_data[7];
    motor1_amps = DRPS_Union.fvalue;
  }

  Wire.endTransmission();
}
