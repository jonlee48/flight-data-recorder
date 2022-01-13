/* George Washington University
 * Design Build Fly
 * October 2019
 * Jonathan Lee
 * Based off code developed by Adafruit
 */

// LIBRARY SETUP -------------------------------------------------------------------------------------
#include <Wire.h>             // I2C lib for IMU
#include <SD.h>               // SD lib
#include <SPI.h>              // wiring protocol
#include <Adafruit_Sensor.h>  // Adafruit sensor lib
#include <Adafruit_BNO055.h>  // IMU lib
#include <utility/imumaths.h> // IMU linear acceleration
#include <Adafruit_BMP3XX.h>  // Altimeter lib
#include <Adafruit_GPS.h>     // for GPS parsing

// SETTINGS ------------------------------------------------------------------------------------------
#define PERIOD_MS   100       // time between measurements, milliseconds
#define BAUDRATE    115200    // serial port baud rate (bits per second)

// HARDWARE PINS -------------------------------------------------------------------------------------
// IMU:
#define BNO_RST A3        // reset pin not used
// ALTIMETER:
#define BMP_SCK A5
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
// SD:
#define cardSelect 4      // M0 SD card
//PITOT:
#define PITOT A0          // Pitot pressure differential
// GPS:
#define GPSSerial Serial1
// BUTTONS & LEDS:
#define BUTTON_TARE 11    // Tactile TARE button, connects to gnd
#define BUTTON_SD 10      // Tactile SD button, connects to gnd
#define SD_LED 6          // Blue LED - solid on indicates writing to SD
#define STATUS 5          // Green LED - blinking indicates sensors collecting data
#define FAILED 9          // Red LED - solid on indicates a system failure (bad)

// GLOBAL VARIABLES ----------------------------------------------------------------------------------
Adafruit_BNO055 bno;      // declare IMU
Adafruit_BMP3XX bmp;      // declare altimeter
Adafruit_GPS GPS(&GPSSerial); // declare GPS
File logfile;             // stores txt file to write to
imu::Vector<3> v;         // stores linear accel vector

char filename[15];        // char arr of name of txt file
unsigned long tprev;      // previous loop time
float groundAlt;          // used to calc change in elevation
boolean LEDstate;         // green STATUS LED
boolean SDstate;          // when true, write to SD
boolean prevSD = false;

// IMU:
float heading, pitch, roll;

// Altimeter:
float tempF, pres, deltaAlt;

// Pitot Tube:
float adc_avg = 0; 
float veloc = 0.0;
float V_0 = 3.3;          // supply voltage to the pressure sensor
float rho = 1.204;        // density of air 
// parameters for averaging and offset
int offset = 0;
int offset_size = 10;
int veloc_mean_size = 20;
int zero_span = 2;

// GPS:
String GPSstr;

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

// VOID SETUP ========================================================================================
void setup(void)
{
  Serial.begin(BAUDRATE); // serial print for diagnostics

  // Setup pins
  pinMode(BUTTON_TARE, INPUT_PULLUP);
  pinMode(BUTTON_SD, INPUT_PULLUP);
  pinMode(SD_LED, OUTPUT);
  pinMode(STATUS, OUTPUT);
  pinMode(FAILED, OUTPUT);

  // GPS SETUP ---------------------------------------------------------------------------------------
  GPSSerial.begin(9600);  // default rate for the Ultimate GPS
  // Tell GPS to output only GGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // And send 10 updates per second
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);


  // PITOT TUBE SETUP --------------------------------------------------------------------------------
  for (int ii=0;ii<offset_size;ii++){
    offset += analogRead(PITOT)-(1023/2);
  }
  offset /= offset_size;

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
  
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    analogWrite(FAILED, 25);
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
  if( ! logfile ) {
//    Serial.print("Couldnt create "); 
//    Serial.println(filename);
    analogWrite(FAILED, 25);
  }

//  Serial.print("Writing to "); 
//  Serial.println(filename);
//  Serial.println("Ready!");

  // Set current runtime
  tprev = millis();
}

// IMU DATA ------------------------------------------------------------------------------------------
void getIMU() {
  // var for lin acceleration
  v = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  
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
  heading = 180/M_PI * atan2(q.x()*q.y() + q.w()*q.z(), 0.5 - q.y()*q.y() - q.z()*q.z());
  pitch   = 180/M_PI * asin(-2.0 * (q.x()*q.z() - q.w()*q.y()));
  roll    = 180/M_PI * atan2(q.w()*q.x() + q.y()*q.z(), 0.5 - q.x()*q.x() - q.y()*q.y());
  heading = heading < 0 ? heading+360 : heading;  // wrap heading to 0 to 360 convention
}

// ALTIMETER DATA ------------------------------------------------------------------------------------
void getAltimeter() {
  tempF = bmp.temperature * 9/5 + 32;                             //  in degrees F
  pres = bmp.pressure / 100.0;                                    // in hPa
  deltaAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA) - groundAlt;  // in meters
}

// PITOT TUBE DATA -----------------------------------------------------------------------------------
void getPitot() {
  // average a few ADC readings for stability
  for (int ii=0;ii<veloc_mean_size;ii++){
    adc_avg+= analogRead(PITOT)-offset;
  }
  adc_avg/=veloc_mean_size;
  
  // make sure if the ADC reads below 512, then we equate it to a negative velocity
  if (adc_avg>512-zero_span and adc_avg<512+zero_span){
  } else{
    if (adc_avg<512){
      veloc = -sqrt((-10000.0*((adc_avg/1023.0)-0.5))/rho);
    } else{
      veloc = sqrt((10000.0*((adc_avg/1023.0)-0.5))/rho);
    }
  }
}

// GPS DATA ------------------------------------------------------------------------------------------
void getGPS() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    GPSstr = GPS.lastNMEA();
    //Serial.print(GPSstr);
    if (!GPS.parse(GPS.lastNMEA())) // if checksum fails, don't print the data
      GPSstr = "\n";
  }
}

// PRINT SENSOR DATA ---------------------------------------------------------------------------------
void printSensors() {
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
    Serial.print(v.y());  //y accel
    Serial.print(",");
    Serial.print(v.z());  //z accel

    // ALTIMETER DATA --------------------------------------------------------------------------------
    Serial.print(tempF);      //degrees, F
    Serial.print(F(" "));
    Serial.print(pres);       // pressure, hPa
    Serial.print(F(" "));
    Serial.print(deltaAlt);   // altitude, meters
    Serial.print(F(" "));
    Serial.print(veloc);      // airspeed m/s
    Serial.print(F(" "));
    Serial.print(GPSstr);
    //Serial.println(F(""));
  }
}

// WRITE SENSOR DATA TO FILE -------------------------------------------------------------------------
void writeToFile() {
  if (heading == heading) {
    logfile.print(millis()/1000.0);
    logfile.print(F(" "));
    logfile.print(heading);
    logfile.print(F(" "));
    logfile.print(pitch);  
    logfile.print(F(" "));
    logfile.print(roll);   
    logfile.print(F(" "));
    logfile.print(v.x());   //x acceleration
    logfile.print(F(" "));
    logfile.print(v.y());   //y accel
    logfile.print(F(" "));
    logfile.print(v.z());   //z accel
    logfile.print(F(" "));
    logfile.print(tempF);
    logfile.print(F(" "));
    logfile.print(pres);
    logfile.print(F(" "));
    logfile.print(deltaAlt);
    logfile.print(F(" "));
    logfile.print(veloc);
    logfile.print(F(" "));
    logfile.print(GPSstr);
    //logfile.println(F(""));
    logfile.close(); // close file to save new line
  }
}

 // VOID LOOP ========================================================================================
void loop(void) {

  getGPS(); // this needs to be called every loop
  
  if (millis() - tprev >= PERIOD_MS) {    // only run this part 10 times a second
    // retrieve data from sensors
    getIMU();
    getAltimeter();
    getPitot();

    // Blink green status LED on 1 cycle and off next cycle
    if (LEDstate) {analogWrite(STATUS,25);}
    else {analogWrite(STATUS,0);}
    LEDstate = !LEDstate;
    
    // Uncomment for diagnostics
    //printSensors();
    
    // Change SDstate if button is pressed
    if(!digitalRead(BUTTON_SD)) {
      SDstate = !SDstate;
      delay(1000); // debounces button & prevents double counting
    }
  
    // WRITE SERIAL DATA TO TEXT FILE VIA SD CARD ------------------------------------------------------
    if(SDstate) { //writes to SD if true
      if(!prevSD) {
        prevSD = true;
        logfile.print("Millis: Heading: Pitch: Roll: Xaccel: Yaccel: Zaccel: Temp(F): Pres(hPa): Alt(m): Pitot(m/s): GPS(NMEA_GGA)\n");
      }
      logfile = SD.open(filename, FILE_WRITE); // try opening test.txt
         analogWrite(SD_LED,100);
         if(logfile) { // write current serial data if txt file is open
             writeToFile();
         }
         else { // couldn't open txt file, turn of SD_LED
          analogWrite(SD_LED,0);
         }  
      }
    else { // keep SD_LED off if BUTTON_SD is not pressed
      analogWrite(SD_LED,0);  
    }
  
    tprev += PERIOD_MS; // update previous time
  }
}
