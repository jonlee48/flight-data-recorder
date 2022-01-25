/*
Outputs data in csv format with 27 values per line, 12 sensor readings + GPS NMEA GPGGA sentence
Count, System Calibration level (0-3), Linear Acceleration XYZ (m/s^2), Gyro XYZ (radians/sec), Quaternion WXYZ, GPS NMEA GPGGA sentence
*/

/* 
 * Inline documentation:
 * BUTTON - used to start & stop recording
 * GREEN_LED - calibration status (on: good, off: bad)
 * RED_LED - blinks when sending data/recording
 */

#include <SD.h> 
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <RH_RF95.h>

// Tunable parameters:
#define SAMPLE_RATE 100   // sample rate in milliseconds
#define POWER 23          // transmitter power from 5 to 23 dBm (default is 13 dBm)

// Probably never change these:
#define GPS_SERIAL Serial1// GPS TX/RX
#define BNO_ADDR 0x29     // IMU I2C address (0x28 or 0x29)
#define CARD_SELECT 4     // SD card pin
#define BUTTON A5         // Momentary button
#define RED_LED 13        // Built-in LED
#define GREEN_LED 8       // Built-in LED
#define RFM95_CS A1       // LoRa CS pin
#define RFM95_RST A3      // LoRa RST pin
#define RFM95_INT A2      // LoRa INT pin
#define RF95_FREQ 915.0   // LoRa frequency (MHz)


// Create instances of sensor classes
Adafruit_GPS GPS(&GPS_SERIAL);
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO_ADDR);
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Declare global variables
unsigned long prev_time = 0;
unsigned long count = -1;
File logfile;
char filename[15];
String GPSstr;
uint8_t sys, gyro, accel, mag = 0;

// for button
boolean record = false;
unsigned long record_last = 0;
const int debounce = 1500;

int16_t packetnum = 0;  // packet counter, we increment per transmission


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
  delay(500);
  Serial.println("Hello???");

  // GPS SETUP
  GPS_SERIAL.begin(9600);
  // Output only GGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // And send 10 updates per second
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);


  // BNO SETUP
  if(!bno.begin())
  {
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1) {
      blinkError();
    }
  }
  bno.setExtCrystalUse(true);


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


  // Setup completed successfully
  digitalWrite(GREEN_LED, HIGH);
}


/* 
 *  Blink green led to signify error
 * Setup failed
 */
void blinkError() {
  int rate = 100;
  digitalWrite(GREEN_LED, HIGH);
  delay(rate);
  digitalWrite(GREEN_LED, LOW);
  delay(rate);
}


/* 
 * Get GPS string
 */
void getGPS() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPSstr = GPS.lastNMEA();
    //Serial.print(GPSstr);
    if (!GPS.parse(GPS.lastNMEA())) // if checksum fails, don't print the data
      GPSstr = "";
  }
}




void loop(void) {
  
  //getGPS(); // this needs to be called every loop
  
  /* Display calibration status for each sensor. */
  //bno.getCalibration(&sys, &gyro, &accel, &mag);
  /*
  // turn on green light if imu is calibrated
  if (sys > 0)
    digitalWrite(GREEN_LED, HIGH);
  else
    digitalWrite(GREEN_LED, LOW);

  
  // if new record button is pressed
  if (digitalRead(BUTTON) == LOW && millis() >= record_last + debounce) {
    record_last = millis();
    record = !record;
    if (record) {
      Serial.println("start recording");
      count = -1;
      // create new FLIGHTXX.txt
      strcpy(filename, "/FLIGHT00.txt");
      for (uint8_t i = 0; i < 100; i++) {
        filename[7] = '0' + i/10;
        filename[8] = '0' + i%10;
        if (! SD.exists(filename)) {
          break;
        }
      }
      // open file
      logfile = SD.open(filename, FILE_WRITE);
    }
    else {
      Serial.println("stop recording");
    }
  }

  // only print if it's time to sample and record is pressed
  if (millis() >= prev_time + SAMPLE_RATE && record) {
    prev_time = millis();

 
    
    imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> rps = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Quaternion quat = bno.getQuat();

    // count (0++)
    Serial.print(++count); Serial.print(",");
    // calibration level (0-3)
    Serial.print(sys, DEC); Serial.print(",");
    // linear acceleration XYZ (m/s^2)
    Serial.print(lin.x()); Serial.print(",");
    Serial.print(lin.y()); Serial.print(",");
    Serial.print(lin.z()); Serial.print(",");
    // gyroscope XYZ (rad per sec)
    Serial.print(rps.x()); Serial.print(",");
    Serial.print(rps.y()); Serial.print(",");
    Serial.print(rps.z()); Serial.print(",");
    // Quaternion data WXYZ
    Serial.print(quat.w(), 4); Serial.print(",");
    Serial.print(quat.x(), 4); Serial.print(",");
    Serial.print(quat.y(), 4); Serial.print(",");
    Serial.print(quat.z(), 4); Serial.print(",");
    Serial.println(GPSstr);
    
    
    logfile = SD.open(filename, FILE_WRITE);
    if (logfile) {
      logfile.print(count); logfile.print(",");
      // calibration level (0-3)
      logfile.print(sys, DEC); logfile.print(",");
      // linear acceleration XYZ (m/s^2)
      logfile.print(lin.x()); logfile.print(",");
      logfile.print(lin.y()); logfile.print(",");
      logfile.print(lin.z()); logfile.print(",");
      // gyroscope XYZ (rad per sec)
      logfile.print(rps.x()); logfile.print(",");
      logfile.print(rps.y()); logfile.print(",");
      logfile.print(rps.z()); logfile.print(",");
      // Quaternion data WXYZ
      logfile.print(quat.w(), 4); logfile.print(",");
      logfile.print(quat.x(), 4); logfile.print(",");
      logfile.print(quat.y(), 4); logfile.print(",");
      logfile.print(quat.z(), 4);logfile.print(",");
      logfile.print(GPSstr);
      logfile.close();

   
    }

  }
  */
  if (millis() >= prev_time + SAMPLE_RATE) {
    prev_time = millis();
    
    // SENDING PACKET OVER RADIO
    count++;
    digitalWrite(RED_LED, HIGH);
  
  
    // Radio packet buffer
    char radiopacket[200];
    memset(radiopacket,0,200);
  
    // Sensor reading buffer
    char bufWord[100];
    memset(bufWord,0,100);
    
    // get data from IMU
    imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> rps = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Quaternion quat = bno.getQuat();
  
    String t = String(millis());
    t.toCharArray(bufWord,100);
    strcat(radiopacket,bufWord);
    strcat(radiopacket,",");
  
    String c = String(count);
    c.toCharArray(bufWord,100);
    strcat(radiopacket,bufWord);
    strcat(radiopacket,",");
  
    // linear acceleration XYZ (m/s^2)
    String linx = String(lin.x());
    linx.toCharArray(bufWord,100);
    strcat(radiopacket,bufWord);
    strcat(radiopacket,",");
  
    String liny= String(lin.y());
    liny.toCharArray(bufWord,100);
    strcat(radiopacket,bufWord);
    strcat(radiopacket,",");
  
    String linz= String(lin.z());
    linz.toCharArray(bufWord,100);
    strcat(radiopacket,bufWord);
  
    Serial.println(radiopacket);

    // blink
    digitalWrite(RED_LED, LOW);
    
    rf95.send((uint8_t *)radiopacket, 30); // LIMIT 30 chars
    rf95.waitPacketSent();

  }

  /*
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(1000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  digitalWrite(RED_LED, LOW);
  */
 
}
