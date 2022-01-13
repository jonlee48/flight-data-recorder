/* George Washington University
 * Design Build Fly
 * October 2019
 * Jonathan Lee
 * Based off code developed by Adafruit
 */

// LIBRARY SETUP -------------------------------------------------------------------------------------
#include <Wire.h> // I2C library for IMU
#include <SD.h> // SD library
#include <SPI.h> // need library for SD wiring
#include "SdFat.h"
#include <Adafruit_Sensor.h> // General Adafruit sensor library
#include <Adafruit_BNO055.h> // IMU library
#include <utility/imumaths.h> // for linear acceleration
#include <Adafruit_BMP3XX.h> // Altimeter library

// SETTINGS ------------------------------------------------------------------------------------------
#define PERIOD_MS   100    // time between measurements, milliseconds
#define BAUDRATE    115200  // serial port baud rate (bits per second)

// HARDWARE PINS ------------------------------------------------------------------------------------------
// IMU
#define BNO_RST     A3     // pin number of BNO055 RST

// ALTIMETER:
#define BMP_SCK A5
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

// SD:
#define SD_CD 10 //SS

// BUTTONS & LEDS:
#define BUTTON_TARE 11   // pin number of TARE button, button connects to ground
#define BUTTON_SD 9     // pin number of SD button, button connects to ground
#define SD_LED 6        // solid blue LED/off
#define STATUS 5        // green blinking LED/off

// GLOBAL VARIABLES ----------------------------------------------------------------------------------
Adafruit_BNO055 bno; // declare IMU
Adafruit_BMP3XX bmp; // declare altimeter
//SdFat SD;
File myFile; // stores open file
imu::Vector<3> v; // stores linear accel vector

unsigned long tprev; // stores previous time
float groundAlt; // reference to ground altitude, needed to calc elevation off ground
boolean LEDstate; // for green STATUS LED
boolean SDstate; // when true, write to SD

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif
// VOID SETUP ========================================================================================
void setup(void)
{
  Serial.begin(BAUDRATE); // begin serial communication

  // Setup pins
  pinMode(BUTTON_TARE, INPUT_PULLUP);
  pinMode(BUTTON_SD, INPUT_PULLUP);
  pinMode(SD_LED, OUTPUT);
  pinMode(STATUS, OUTPUT);
  pinMode(10, OUTPUT);

  // IMU SETUP ---------------------------------------------------------------------------------------
  Wire.begin();
  Wire.setClock(100000L);           // I2C speed
  digitalWrite(BNO_RST,0);
  pinMode(BNO_RST, OUTPUT);         // assert BNO RST
  delay(2);                         // 1 ms is ample but some microcontrollers truncate the first ms
  pinMode(BNO_RST, INPUT_PULLUP);   // deassert BNO RST
  delay(800);                       // allow processor on BNO to boot

  //Compiler instructions if mode is defined
  #if 1    // my BNO is mounted with its dot rearward/right/up so choose P3 (see BNO055 datasheet)
    bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);
    bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);
  #endif
  bno.setMode(bno.OPERATION_MODE_NDOF);
  delay(10);                        // allow time for BNO to switch modes

  // ALTIMETER SETUP ---------------------------------------------------------------------------------
  bmp.begin();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);

  // SD SETUP ----------------------------------------------------------------------------------------
  SPI.begin();
  SD.begin(SD_CD);

  // Set current runtime
  tprev = millis();
}


 // VOID LOOP ========================================================================================
void loop(void)
{
  // var for lin acceleration
  v = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  // Blink green status LED on 1 cycle and off next cycle
  if (LEDstate) {analogWrite(STATUS,25);}
  else {analogWrite(STATUS,0);}
  LEDstate = !LEDstate;
  
  //IMU GYROSCOPE DATA -----------------------------------------------------------------------------------------
  imu::Quaternion q = bno.getQuat();
  // flip BNO/Adafruit quaternion axes to aerospace: x forward, y right, z down
  float temp = q.x();  q.x() = q.y();  q.y() = temp;  q.z() = -q.z();
  static imu::Quaternion tare = {1,0,0,0};
  // If tare button is pressed:
  if (!digitalRead(BUTTON_TARE)) {
    tare = q.conjugate();
    groundAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  // Compiler instructions:
  #if 0   // North-referenced (resets to magnetic north)
    q = q * tare;
  #else   // Screen-referenced (resets to current orientation)
    q = tare * q;
  #endif
  q.normalize();
  
  // convert aerospace quaternion to aerospace Euler, because BNO055 Euler data is broken
  float heading = 180/M_PI * atan2(q.x()*q.y() + q.w()*q.z(), 0.5 - q.y()*q.y() - q.z()*q.z());
  float pitch   = 180/M_PI * asin(-2.0 * (q.x()*q.z() - q.w()*q.y()));
  float roll    = 180/M_PI * atan2(q.w()*q.x() + q.y()*q.z(), 0.5 - q.x()*q.x() - q.y()*q.y());
  heading = heading < 0 ? heading+360 : heading;  // wrap heading to 0 to 360 convention

  // SERIAL PRINT SPACE DELIMITED SENSOR READINGS ----------------------------------------------------
  if (heading == heading) { // fliters out NaN (not a number) readings
    Serial.print(millis()/1000.0); // time since start, seconds
    Serial.print(F(" "));
    Serial.print(heading);  // heading, nose-right is positive, degrees
    Serial.print(F(" "));
    Serial.print(pitch);    // pitch, nose-up is positive, degrees
    Serial.print(F(" "));
    Serial.print(roll);     // roll, leftwing-up is positive, degrees
    Serial.print(F(" "));

    //IMU ACCELERATION DATA --------------------------------------------------------------------------
    Serial.print(v.x());  //x acceleration
    Serial.print(",");
    Serial.print(v.y()); //y accel
    Serial.print(",");
    Serial.print(v.z()); //z accel

    // ALTIMETER DATA --------------------------------------------------------------------------------
    Serial.print(bmp.temperature * 9/5 + 32); //degrees, F
    Serial.print(F(" "));
    Serial.print(bmp.pressure / 100.0); // pressure, hPa
    Serial.print(F(" "));
    Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) - groundAlt); // altitude, meters
    Serial.println(F(""));
    
  }

  // Change SDstate if button is pressed
  if(!digitalRead(BUTTON_SD)) {
    SDstate = !SDstate;
    delay(1000); // prevents double counting
  }

  // WRITE SERIAL DATA TO TEXT FILE VIA SD CARD ------------------------------------------------------
  if(SDstate) { //writes to SD if true
    myFile = SD.open("TEST.txt", FILE_WRITE); // try opening test.txt
       analogWrite(SD_LED,100);
       if(myFile) { // write current serial data if txt file is open
           if (heading == heading) {
              myFile.print(millis()/1000.0);
              myFile.print(F(" "));
              myFile.print(heading);
              myFile.print(F(" "));
              myFile.print(pitch);  
              myFile.print(F(" "));
              myFile.print(roll);   
              myFile.print(F(" "));
              myFile.print(v.x());  //x acceleration
              myFile.print(F(" "));
              myFile.print(v.y()); //y accel
              myFile.print(F(" "));
              myFile.print(v.z()); //z accel
              myFile.print(F(" "));
              myFile.print(bmp.temperature * 9/5 + 32);
              myFile.print(F(" "));
              myFile.print(bmp.pressure / 100.0);
              myFile.print(F(" "));
              myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA) - groundAlt);
              myFile.println(F(""));
              myFile.close(); // close file to save new line
            }
       }
       else { // couldn't open txt file, turn of SD_LED
        analogWrite(SD_LED,0);
       }  
    }
  else { // keep SD_LED off if BUTTON_SD is not pressed
       analogWrite(SD_LED,0);  
  }
  
  while (millis() - tprev < PERIOD_MS);  // hold here until time interval passes
  tprev += PERIOD_MS; // update previous time
  // keep looping
}
