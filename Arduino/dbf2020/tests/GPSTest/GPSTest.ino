// Test code for Ultimate GPS Using Hardware Serial
// (e.g. GPS for Leonardo, Flora or FeatherWing)
//
// This code shows how to test a passthru between USB and hardware serial
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
//
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada


#include <Adafruit_GPS.h>

// what's the name of the hardware serial port?
#define GPSSerial Serial1


void setup() {
  // wait for hardware serial to appear
  //while (!Serial);
  Adafruit_GPS GPS(&GPSSerial);
  // make this baud rate fast enough to we aren't waiting on it
  Serial.begin(115200);

  // 9600 baud is the default rate for the Ultimate GPS
  GPSSerial.begin(9600);
  // Tell GPS to output only GGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  // Tell GPS to send 10 updates a second
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
}


void loop() {
//  if (Serial.available()) {
//    char c = Serial.read();
//    GPSSerial.write(c);
//  }
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    Serial.write(c);
  }
  Serial.println("----");
}
