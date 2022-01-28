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
#define POWER 23          // transmitter power from 5 to 23 dBm (default is 13 dBm)

// Probably never change these:
#define GPS_SERIAL Serial1// GPS TX/RX
#define BNO_ADDR 0x29     // IMU I2C address (0x28 or 0x29)
#define CARD_SELECT 4     // SD card pin
#define BUTTON A5         // Momentary button
#define DEBOUNCE 1000     // Milliseconds between registering presses
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

// Declare global variables
unsigned long prev_time, count;
File logfile;
char filename[15];
String GPSstr;
uint8_t sys, gyro, accel, mag;

// for button
boolean record = false;
unsigned long record_last;
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
  delay(1000);
  //while(!Serial); // Waits until serial monitor is open
  

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
    if (!GPS.parse(GPS.lastNMEA())) { // if checksum fails, don't print the data
      GPSstr = "\n";
    }
    else {
      GPSstr = GPS.lastNMEA();
    }
  }
}

/*
 * Records sensor reading into Serial and logfile, adding comma after
 * Data type double
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
 * Data type double
 */
void sensorPrintDou(double data, File logfile) {
  Serial.print(data);
  Serial.print(",");
  
  if (logfile) {
    logfile.print(data);
    logfile.print(",");
  }
}

/*
 * Records sensor reading into Serial and logfile, adding newline after
 * Data type String
 */
void sensorPrintStr(String str, File logfile) {
  Serial.print(str);
    
  if (logfile) {
    logfile.print(str);
  }
}


void loop(void) {

  // read from sensors
  getGPS();
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> rps = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();
  bmp.performReading();
  
  
  // turn on green light if imu is calibrated
  if (sys > 0) {
    digitalWrite(GREEN_LED, HIGH);
  }
  else {
    digitalWrite(GREEN_LED, LOW);
  }

  
  // if new record button is pressed
  if (digitalRead(BUTTON) == LOW && millis() >= record_last + DEBOUNCE) {
    record_last = millis();
    record = !record;
    if (record) {
      count = 0;
      // create new FLIGHTXX.txt
      strcpy(filename, "/FLIGHT00.txt");
      for (uint8_t i = 0; i < 100; i++) {
        filename[7] = '0' + i/10;
        filename[8] = '0' + i%10;
        if (! SD.exists(filename)) {
          break;
        }
      }
    }
  }

  
  // only print if it's time to sample and record is pressed
  if (record && millis() >= prev_time + SAMPLE_RATE) {
    prev_time = millis();

    // print data from all sensors to Serial and logfile
    logfile = SD.open(filename, FILE_WRITE);

    sensorPrintInt(count, logfile);
    sensorPrintInt(sys, logfile);
    sensorPrintDou(lin.x(), logfile);
    sensorPrintDou(lin.y(), logfile);
    sensorPrintDou(lin.z(), logfile);
    sensorPrintDou(rps.x(), logfile);
    sensorPrintDou(rps.y(), logfile);
    sensorPrintDou(rps.z(), logfile);
    sensorPrintDou(quat.w(), logfile);
    sensorPrintDou(quat.x(), logfile);
    sensorPrintDou(quat.y(), logfile);
    sensorPrintDou(quat.z(), logfile);
    sensorPrintDou(bmp.temperature, logfile);
    sensorPrintDou(bmp.pressure, logfile);
    sensorPrintStr(GPSstr, logfile);

    logfile.close();

  }
}

  /* 
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
}
*/
