/*
Outputs data in csv format with 27 values per line, 12 sensor readings + GPS NMEA GPGGA sentence
Count, System Calibration level (0-3), Linear Acceleration XYZ (m/s^2), Gyro XYZ (radians/sec), Quaternion WXYZ, GPS NMEA GPGGA sentence
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h> 

/* Set the delay between fresh samples */
#define SAMPLE_RATE (10)
// SD:
#define CARD_SELECT 4      // M0 SD card
// GPS:
// Define some I/O buttons and LEDs
#define BUTTON A5
#define RECORD_LED 13
#define STATUS_LED 8


// Global Vars
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
unsigned long prev_time = 0;
unsigned long count = -1;
File logfile;
char filename[15];
uint8_t sys, gyro, accel, mag = 0;


// for button
boolean record = false;
unsigned long record_last = 0;
const int debounce = 1500;

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

  digitalWrite(RECORD_LED, LOW);

  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1); // Keep from going to void loop()
  }

  // SD SETUP ************************************************************/
  // see if the card is present and can be initialized:
  if (!SD.begin(CARD_SELECT)) {
    Serial.println("Card init. failed!");
    while(1);
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
}


/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/

void loop(void) {
  
  
  /* Display calibration status for each sensor. */
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  
  // turn on green light if imu (bno) is calibrated
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
      digitalWrite(RECORD_LED, HIGH);
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
      digitalWrite(RECORD_LED, LOW);
    }
  }

  // only print if it's time to sample and record is pressed
  if (millis() >= prev_time + SAMPLE_RATE && record) {
    prev_time = millis();
    
    imu::Vector<3> lin = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> rps = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Quaternion quat = bno.getQuat();

    // count (0++)
//    Serial.print(++count); Serial.print(",");
//    // calibration level (0-3)
//    Serial.print(sys, DEC); Serial.print(",");
//    // linear acceleration XYZ (m/s^2)
    Serial.print("49,");
    Serial.print(lin.x()); Serial.println(",");
    //Serial.print(lin.y()); Serial.print(",");
    //Serial.print(lin.z()); Serial.println(",");
    // gyroscope XYZ (rad per sec)
//    Serial.print(rps.x()); Serial.print(",");
//    Serial.print(rps.y()); Serial.print(",");
//    Serial.print(rps.z()); Serial.print(",");
//    // Quaternion data WXYZ
//    Serial.print(quat.w(), 4); Serial.print(",");
//    Serial.print(quat.x(), 4); Serial.print(",");
//    Serial.print(quat.y(), 4); Serial.print(",");
//    Serial.print(quat.z(), 4); Serial.println(",");
    
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
      logfile.print(quat.z(), 4);logfile.println(",");
      logfile.close();
    }

  }
}
