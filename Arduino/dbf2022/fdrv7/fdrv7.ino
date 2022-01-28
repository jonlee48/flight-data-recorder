/*
Outputs data in csv format with 27 values per line, 12 sensor readings + GPS NMEA GPGGA sentence
Count, System Calibration level (0-3), Linear Acceleration XYZ (m/s^2), Gyro XYZ (radians/sec), Quaternion WXYZ, GPS NMEA GPGGA sentence
*/

/* 
 * Inline documentation:
 * BUTTON - used to start & stop recording
 * GREEN_LED - solid on if calibrated, blinks upon initialization error
 * RED_LED - blinks when sending data/recording
 */

#include <SD.h> 
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <RH_RF95.h>

// Tunable parameters:
#define SAMPLE_RATE 100   // sample rate in milliseconds
#define SEALEVELPRESSURE_HPA 1026.753  // Depends on weather. Default is 1013.25
#define POWER 23          // transmitter power from 5 to 23 dBm (default is 13 dBm)
#define NUM_AVG 20        // how many data points are in the pitot tube moving average

// Probably never change these:
#define GPS_SERIAL Serial1// GPS TX/RX
#define BNO_ADDR 0x29     // IMU I2C address
#define PITOT_ADDR 0x28   // MS4525 sensor I2C address
#define PACKET_SIZE RH_RF95_MAX_MESSAGE_LEN  // Max radio packet size (251 bytes)
#define CARD_SELECT 4     // SD card pin
#define BUTTON A5         // Momentary button
#define DEBOUNCE 1500     // Milliseconds between registering presses
#define RED_LED 13        // Built-in LED
#define GREEN_LED 8       // Built-in LED
#define RFM95_CS A1       // LoRa CS pin
#define RFM95_RST A3      // LoRa RST pin
#define RFM95_INT A2      // LoRa INT pin
#define RF95_FREQ 915.0   // LoRa frequency (MHz)


// Create instances of sensor classes
Adafruit_GPS GPS(&GPS_SERIAL);                        // GPS
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO_ADDR);  // IMU
Adafruit_BMP3XX bmp;                                  // Altimeter
RH_RF95 rf95(RFM95_CS, RFM95_INT);                    // LoRa

// Global variables (initialized to 0)
unsigned long prev_sample;      // Time in millis of last sample
unsigned long count;            // Sample counter
unsigned long prev_press;       // Time in millis of last button press
unsigned long packetnum;        // Packet counter
unsigned int p_pres;            // Pitot tube raw pressure
unsigned int p_temp;            // Pitot tube raw temp
double p_psi;                   // Pitot tube pressure (psi)
double p_vel;                   // Pitot tube velocity
double p_avg;                   // Pitot tube average velocity (avg of last X data points)
double p_cel;                   // Pitot tube temp (*C)
int p_stat;                     // Pitot tube status 0: good 1, 2: error
double avg[NUM_AVG];            // Pitot tube velocity moving average
double b_pres;                  // Altimeter pressure (hPa)
double b_alt;                   // Altimeter meters above sea level
File logfile;                   // File object to store open file
char fname[] = "/FLIGHT00.TXT"; // Filename, chars 7,8 are incremented
String GPSstr = "\n";           // GPS NMEA formatted string
uint8_t sys;                    // IMU overall status [0-3] uncalibrated->fully calibrated
uint8_t gyro;                   // IMU gyroscope status [0-3]
uint8_t accel;                  // IMU accelerometer status [0-3]
uint8_t mag;                    // IMU magnetometer status [0-3]
boolean record;                 // Record data - state changed by button


void setup(void)
{
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(RFM95_RST, OUTPUT);
  
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  Serial.begin(115200);
  //while(!Serial);  // Waits until serial monitor is open


  // PITOT SETUP
  Wire.begin();
  delay(500);
  

  // GPS SETUP
  GPS_SERIAL.begin(9600);
  // Output only GGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // And send 1 update per second (fastest GPS can go)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);


  // BNO SETUP
  if(!bno.begin())
  {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1) {
      blinkError();
    }
  }
  bno.setExtCrystalUse(true);


  // BMP SETUP
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1) {
      blinkError();
    }
  }
  // Set up oversampling and filter initialization
  // See Filter Selection, page 16: https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP388-DS001.pdf
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X); // 0.0006 *C resolution
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);    // 0.66 Pa resolution
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);


  // SD SETUP
  // see if the card is present and can be initialized:
  if (!SD.begin(CARD_SELECT)) {
    Serial.println("Card init. failed!");
    while(1) {
      blinkError();
    }
  }


  // LoRa SETUP
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while(1) {
      blinkError();
    }
  }

  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(POWER, false);
}


void loop(void) {

  // register new button press
  if (digitalRead(BUTTON) == LOW && millis() >= prev_press + DEBOUNCE) {
    prev_press = millis();
    record = !record;  // toggle state
    
    if (record) {
      start_record();
    }
  }

  
  // only record if it's time to sample
  if (record && millis() >= prev_sample + SAMPLE_RATE) {
    prev_sample = millis();

    // read from sensors
    getGPS();
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> rps = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Quaternion quat = bno.getQuat();
    
    bmp.performReading();
    p_stat = fetch_pitot(&p_pres, &p_temp);

    // pitot tube conversions
    p_psi = (double)((p_pres - 819.15) / 14744.7);
    p_psi = p_psi - 0.49060678;
    p_psi = abs(p_psi);
    
    p_vel = ((p_psi * 13789.5144) / 1.225);
    p_vel = sqrt(p_vel);

    p_cel = (double)(p_temp * 0.09770395701);
    p_cel = p_cel - 50;

    p_avg = moving_avg(p_vel);

    // altimeter
    b_pres = bmp.pressure / 100.0;
    b_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    // turn on green light if IMU is calibrated
    if (sys > 0) {
      digitalWrite(GREEN_LED, HIGH);
    }
    else {
      digitalWrite(GREEN_LED, LOW);
    }


    // print data from all sensors to Serial and logfile
    logfile = SD.open(fname, FILE_WRITE);
    
    sensorPrintULo(prev_sample, logfile);
    sensorPrintULo(count, logfile);
    sensorPrintInt(sys, logfile);
    sensorPrintInt(p_stat, logfile);
    sensorPrintDou(p_pres, 1, logfile);
    sensorPrintDou(p_temp, 1, logfile);
    sensorPrintDou(p_psi, 10, logfile);
    sensorPrintDou(p_vel, 3, logfile);
    sensorPrintDou(p_cel, 3, logfile);
    sensorPrintDou(lin.x(), 3, logfile);
    sensorPrintDou(lin.y(), 3, logfile);
    sensorPrintDou(lin.z(), 3, logfile);
    sensorPrintDou(rps.x(), 3, logfile);
    sensorPrintDou(rps.y(), 3, logfile);
    sensorPrintDou(rps.z(), 3, logfile);
    sensorPrintDou(euler.x(), 3, logfile);
    sensorPrintDou(euler.y(), 3, logfile);
    sensorPrintDou(euler.z(), 3, logfile);
    sensorPrintDou(quat.w(), 3, logfile);
    sensorPrintDou(quat.x(), 3, logfile);
    sensorPrintDou(quat.y(), 3, logfile);
    sensorPrintDou(quat.z(), 3, logfile);
    sensorPrintDou(bmp.temperature, 3, logfile);
    sensorPrintDou(b_pres, 3, logfile);
    sensorPrintDou(b_alt, 3, logfile);
    sensorPrintStr(GPSstr, logfile);

    logfile.close();


    
    // SENDING PACKET OVER RADIO
    digitalWrite(RED_LED, HIGH);
  
  
    // Radio packet buffer
    char radiopacket[PACKET_SIZE];
    memset(radiopacket,0,PACKET_SIZE);

    /* PACKET CONTENTS:
     * COUNT
     * PREV_SAMPLE
     * AIRSPEED (moving average)
     * ALTITUDE
     * PITCH
     * ROLL
     * LINX
     * RSSI
     */
    sprintf(radiopacket, "%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,", 
      count,
      prev_sample,
      p_avg,
      b_alt,
      euler.y(),
      euler.z(),
      lin.x()
    );
      
  
    Serial.println(radiopacket);
    Serial.println(strlen(radiopacket));

    // blink
    digitalWrite(RED_LED, LOW);

    int len = min(strlen(radiopacket), PACKET_SIZE);
    
    rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf95.waitPacketSent();


    // update counter
    count++;
  }


  // Allow ground station to start recording
  if (!record) {
    uint8_t buf[20];
    uint8_t len = sizeof(buf);
    if (rf95.waitAvailableTimeout(1000)) { 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len)) {
        if (strcmp((char*)buf, "RECORD") == 0) {
          record = true;
          start_record();
        }
      }
    }
  }
  
  
}


/*
 * Creates new log and resets count
 */
void start_record() {
  count = 0;
  // create new FLIGHTXX.TXT
  for (uint8_t i = 0; i < 100; i++) {
    fname[7] = '0' + i/10;
    fname[8] = '0' + i%10;
    if (!SD.exists(fname)) {
      break;
    }
  }
}


/*
 * Returns sensor status and updates p_pres and p_temp
 */
int fetch_pitot(unsigned int *p_pres, unsigned int *p_temp)
{
  Wire.beginTransmission(PITOT_ADDR);
  Wire.endTransmission();
  delay(10);
  
  // Request 4 bytes = status (2 bits) + pressure (14 bits) + temp (11 bits) + 5 extra bits
  Wire.requestFrom((int)PITOT_ADDR, (int)4);  
  byte Press_H = Wire.read();
  byte Press_L = Wire.read();
  byte Temp_H = Wire.read();
  byte Temp_L = Wire.read();
  Wire.endTransmission();


  // Get status from 2 leftmost bits of Press_H
  byte _status = (Press_H >> 6) & 0x03;

  // Get 14 bit pressure from H & L bytes
  Press_H = Press_H & 0x3f;
  *p_pres = (((unsigned int)Press_H) << 8) | Press_L;

  // Get 11 bit temp from H & L bytes
  Temp_L = (Temp_L >> 5);
  *p_temp = (((unsigned int)Temp_H) << 3) | Temp_L;

  return (int)_status;
}

/*
 * return new moving average by adding a data point to it
 */
double moving_avg(double last) {

  for (int i = NUM_AVG - 1; i > 0; i--) {
    avg[i] = avg[i-1];
  }
  avg[0] = last;

  double sum = 0.0;
  for (int i = 0; i < NUM_AVG; i++) {
    sum += avg[i];
  }

  return sum/NUM_AVG;
}

/* 
 * Update GPSstr
 */
void getGPS() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {    
    if (!GPS.parse(GPS.lastNMEA())) { // if checksum fails, don't print the data
      GPSstr = "\n";
    }
    else {
      GPSstr = GPS.lastNMEA();
    }
  }
}


/* 
 * Blink green led to signify error - setup failed
 */
void blinkError() {
  int rate = 100;
  digitalWrite(GREEN_LED, HIGH);
  delay(rate);
  digitalWrite(GREEN_LED, LOW);
  delay(rate);
}


/*
 * Records sensor reading into Serial and logfile, adding comma after
 */
void sensorPrintULo(unsigned long data, File logfile) {
  Serial.print(data);
  Serial.print(",");
  
  if (logfile) {
    logfile.print(data);
    logfile.print(",");
  }
}


/*
 * Records sensor reading into Serial and logfile, adding comma after
 */
void sensorPrintInt(int data, File logfile) {
  Serial.print(data);
  Serial.print(",");
  
  if (logfile) {
    logfile.print(data);
    logfile.print(",");
  }
}


/*
 * Records sensor reading into Serial and logfile, adding comma after
 */
void sensorPrintDou(double data, int digits, File logfile) {
  Serial.print(data, digits);
  Serial.print(",");
  
  if (logfile) {
    logfile.print(data, digits);
    logfile.print(",");
  }
}


/*
 * Records sensor reading into Serial and logfile
 */
void sensorPrintStr(String str, File logfile) {
  Serial.print(str);
    
  if (logfile) {
    logfile.print(str);
  }
}
