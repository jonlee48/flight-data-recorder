/*
Outputs data in csv format with 27 values per line, 12 sensor readings + GPS NMEA GPGGA sentence
Count, System Calibration level (0-3), Linear Acceleration XYZ (m/s^2), Gyro XYZ (radians/sec), Quaternion WXYZ, GPS NMEA GPGGA sentence
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h> 
#include <Adafruit_GPS.h>

/* Set the delay between fresh samples */
#define SAMPLE_RATE 100
// SD:
#define cardSelect 4      // M0 SD card
// GPS:
#define GPSSerial Serial1
// Define some I/O buttons and LEDs
#define BUTTON A5
#define RECORD_LED 13
#define STATUS_LED 8

// Global Vars
Adafruit_GPS GPS(&GPSSerial);
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
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


#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS A1
#define RFM95_RST A3
#define RFM95_INT A2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Blinky on receipt
#define LED 13

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    Use millis() instead of delay() to account for write and execution times
*/
/**************************************************************************/
void setup(void)
{
  pinMode(STATUS_LED, OUTPUT); // green onboard LED lit if calibrated
  pinMode(RECORD_LED, OUTPUT); // blue LED lit when recording
  pinMode(BUTTON, INPUT_PULLUP); // button to start/stop record

  //Transmitter
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  digitalWrite(RECORD_LED, LOW);

  // GPS SETUP ************************************************************/
  GPSSerial.begin(9600);  // default rate for the Ultimate GPS
  // Tell GPS to output only GGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // And send 10 updates per second
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  // SD SETUP ************************************************************/
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
  }
  delay(1000);
  bno.setExtCrystalUse(true);

delay(100);


Serial.println("Arduino LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  
}
int16_t packetnum = 0;  // packet counter, we increment per xmission

void getGPS() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPSstr = GPS.lastNMEA();
    //Serial.print(GPSstr);
    if (!GPS.parse(GPS.lastNMEA())) // if checksum fails, don't print the data
      GPSstr = "";
  }
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/

void loop(void) {
  
  //getGPS(); // this needs to be called every loop
  
  /* Display calibration status for each sensor. */
  //bno.getCalibration(&sys, &gyro, &accel, &mag);
  /*
  // turn on green light if imu is calibrated
  if (sys > 0)
    digitalWrite(STATUS_LED, HIGH);
  else
    digitalWrite(STATUS_LED, LOW);

  
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
    digitalWrite(RECORD_LED, HIGH);
  
  
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
    digitalWrite(LED, LOW);
    
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
  digitalWrite(RECORD_LED, LOW);
  */
 
}
