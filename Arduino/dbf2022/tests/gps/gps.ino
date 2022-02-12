#include <Adafruit_GPS.h>

#define GPS_SERIAL  Serial1 // GPS TX/RX

Adafruit_GPS GPS(&GPS_SERIAL); 

void setup() {
  Serial.begin(115200);
  
  // GPS SETUP
  GPS_SERIAL.begin(9600);

  // turn on only the "minimum recommended" data for high update rates!
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // 10 Hz update rate - for 9600 baud you'll have to set the output to RMC only (see above)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

}

char gps_buf[100]; // GPS NMEA formatted string
int idx = 0;      // Current index within buffer

void loop() {
  updateGPS();
  //Serial.print(gps_buf);
  Serial.println("doing stuff");
  delay(200);
}

/* 
 * Update gps_buf
 */
void updateGPS() {
  while (GPS_SERIAL.available()) {
    char c = GPS_SERIAL.read();

    // error checking
    if (idx >= 100) {
      memset(gps_buf,0,100);
      idx = 0;
    }
    
    gps_buf[idx] = c;
    idx++;

    // end of $GPRMC NEMA string
    if (c == '\n') {
      Serial.print(gps_buf);
      memset(gps_buf,0,100);
      idx = 0;
    }
  }
}
