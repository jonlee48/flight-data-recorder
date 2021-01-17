/* This reads raw data from the BNO055
Outputs data in csv format with 12 values per line:
Count, System Calibration level (0-3), Linear Acceleration XYZ (m/s^2), Gyro XYZ (radians/sec), Quaternion WXYZ
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h> 

/* Set the delay between fresh samples */
#define SAMPLE_RATE (100)
// SD:
#define cardSelect 4      // M0 SD card
// Define some I/O
#define RECORD_IN A1
#define STATUS_OUT A0
#define BUILTIN_LED 13


// Global Vars
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
unsigned long prev_time = 0;
unsigned long count = -1;
int leds[6] = {5,6,9,10,11,12}; // these correspond to the 6 LED pins
File logfile;
char filename[15];
boolean record = false;
boolean start_record = false;
uint8_t sys, gyro, accel, mag = 0;


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
    Use millis() instead of delay() to account for write and execution times
*/
/**************************************************************************/
void setup(void)
{
  pinMode(RECORD_IN, INPUT); // Audio jack input to start/stop record
  pinMode(STATUS_OUT, OUTPUT); // Audio jack output if calibrated
  pinMode(BUILTIN_LED, OUTPUT); // onboard LED
  
  // set LED control pins
  for (int i=0; i<6; i++) {
    pinMode(leds[i], OUTPUT);
  }

  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  // SD SETUP *************************************************************/
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
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

  if (digitalRead(RECORD_IN) == HIGH) {
    if (record == false) 
      start_record = true;
    else
      start_record = false;
    record = true;
  }
  else {
    record = false;
    start_record = false;
  }

  // output signal through audio jack if calibrated
  if (sys > 0) {
    digitalWrite(STATUS_OUT, HIGH);
    // turn on onboard led to signify it is on
    digitalWrite(BUILTIN_LED, HIGH);
  }
  else {
    digitalWrite(STATUS_OUT, LOW);
    digitalWrite(BUILTIN_LED, LOW);
  }

  if (start_record) {
    count = -1;
    start_record = false;
    // create next FLIGHTXX.txt, don't open existing
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
    if(!logfile) {
      Serial.print("Couldnt create "); 
      Serial.println(filename);
    }
  }

  // only print if it's time to sample and received record signal
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
    Serial.print(quat.w(), 4); Serial.print(",");
    Serial.print(quat.x(), 4); Serial.print(",");
    Serial.print(quat.y(), 4); Serial.print(",");
    Serial.println(quat.z(), 4);
    
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
      logfile.println(quat.z(), 4);
      logfile.close();
    }

  }
}
