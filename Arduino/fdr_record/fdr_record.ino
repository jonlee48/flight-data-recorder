#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h> 
#include <Adafruit_GPS.h>     // for GPS parsing

/* This reads raw data from the BNO055

Outputs data in csv format with 12 values per line:
Count, System Calibration level (0-3), Linear Acceleration XYZ (m/s^2), Gyro XYZ (radians/sec), Quaternion WXYZ


   Connections
   ===========
   Connect SCL to SCL
   Connect SDA to SDA
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   1/4/2021
*/

// SET TO TRUE FOR SERIAL PRINT
#define DEBUG false


/* Set the delay between fresh samples */
#define SAMPLE_RATE (100)
// SD:
#define cardSelect 4      // M0 SD card


// GPS:
#define GPSSerial Serial1

// Global Vars
Adafruit_GPS GPS(&GPSSerial); // declare GPS
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
unsigned long prev_time = 0;
unsigned long count = -1;
boolean initialized = false;
File logfile;
char filename[15];
String GPSstr;

// for buttons
boolean record = false;
unsigned long record_last;
const int debounce = 1500;


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    Use millis() instead of delay() to account for write and execution times
*/
/**************************************************************************/
void setup(void)
{
  pinMode(5, OUTPUT); // if initialized
  pinMode(10, INPUT_PULLUP); // right button (record) on pin 10
  pinMode(6, OUTPUT); // blue LED for record
  pinMode(A4, OUTPUT); // signal for record

  // GPS SETUP ---------------------------------------------------------------------------------------
  GPSSerial.begin(9600);  // default rate for the Ultimate GPS
  // Tell GPS to output only GGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // And send 10 updates per second
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  #endif
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    #ifdef DEBUG
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    #endif
    while(1);
  }

  // SD SETUP ----------------------------------------------------------------------------------------
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    #ifdef DEBUG
    Serial.println("Card init. failed!");
    #endif
  }
  // create next FLIGHTXX.txt, don't open existing
  strcpy(filename, "/FLIGHT00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[7] = '0' + i/10;
    filename[8] = '0' + i%10;
    if (! SD.exists(filename)) {
      break;
    }
  }
  // open file
  logfile = SD.open(filename, FILE_WRITE);
  if(!logfile) {
    #ifdef DEBUG
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    #endif
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);

}

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
void loop(void)
{
    uint8_t sys, gyro, accel, mag = 0;

    if (!initialized && sys > 0) {
    initialized = true;
    digitalWrite(5, HIGH);
  }

  if (!initialized)
    digitalWrite(5, LOW);

  
    if (digitalRead(10) == LOW && millis() >= record_last + debounce) {
    record_last = millis();
    record = !record;
    if (record) {
      Serial.println("start recording");
      digitalWrite(6, HIGH);
      digitalWrite(A4, HIGH);
      count = -1;
        // create next FLIGHTXX.txt, don't open existing
  strcpy(filename, "/FLIGHT00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[7] = '0' + i/10;
    filename[8] = '0' + i%10;
    if (! SD.exists(filename)) {
      break;
    }
  }
  // open file
  logfile = SD.open(filename, FILE_WRITE);
  if(!logfile) {
    #ifdef DEBUG
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    #endif
  }
    }
    else {
      Serial.println("stop recording");
      digitalWrite(6, LOW);
      digitalWrite(A4, LOW);
    }
  }
  
    getGPS(); // this needs to be called every loop
    
    /* Display calibration status for each sensor. */
    bno.getCalibration(&sys, &gyro, &accel, &mag);




  // only print if it's time to sample and the sensor is calibrated and record is pressed
  if (millis() >= prev_time + SAMPLE_RATE && initialized && record) {
    prev_time = millis();
    
    imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> rps = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Quaternion quat = bno.getQuat();

    #ifdef DEBUG
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
    Serial.print(GPSstr);
    #endif
    
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
}
