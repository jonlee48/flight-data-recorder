#include <SD.h> 
#include <SPI.h>
#include <RH_RF95.h>


// Uncomment this line to enable Serial debugging:
//#define DEBUG 1             // Enable debugging

// Tunable parameters:
#define POWER 23          // transmitter power from 5 to 23 dBm (default is 13 dBm)

// Probably never change these:
#define BUTTON A5         // Momentary button
#define DEBOUNCE 1500     // Milliseconds between registering presses
#define RED_LED 13        // Built-in LED
#define GREEN_LED 8       // Built-in LED
#define CARD_SELECT 4     // SD card pin
#define RFM95_CS A1       // LoRa CS pin
#define RFM95_RST A3      // LoRa RST pin
#define RFM95_INT A2      // LoRa INT pin
#define RF95_FREQ 915.0   // LoRa frequency (MHz)

// Instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Global variables (initialized to 0)
unsigned long prev_press;       // Time in millis of last button press
boolean recording;              // Whether recording has started (can't be stopped)
int     event_count;            // Number of events user has logged
File    logfile;                // File object to store open file
char    fname[] = "/FLIGHT00.TXT";  // Filename, chars 7,8 are incremented
const String header =               // header for columns at top of csv file
"time_since_start,\
p_avg,\
bmp_alt,\
euler_y,\
euler_z,\
rssi";

void setup() 
{
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  //while (!Serial);
  delay(500);

  // SD SETUP
  // see if the card is present and can be initialized:
  if (!SD.begin(CARD_SELECT)) {
    #ifdef DEBUG
      Serial.println("Card init. failed!");
    #endif
  }
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    #ifdef DEBUG
      Serial.println("LoRa radio init failed");
    #endif
  }

  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(POWER, false);
}

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];  // 251 bytes
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      digitalWrite(RED_LED, HIGH);
      buf[len] = '\0'; // gets rid of extra junk
      String data = String((char*)buf);
      data.trim();
      data += rf95.lastRssi();
      Serial.println(data);

      if (logfile) {
        logfile.println(data);
      }

      // SAVE A LOG OF MILLIS, RSSI, & EVENTS
      //Serial.println(rf95.lastRssi(), DEC);

      //Serial.println(strlen((char*)buf));
      memset(buf,0,len);
      
      delay(10);
      digitalWrite(RED_LED, LOW);
    }
  }
  // check if record button is pressed
  if (digitalRead(BUTTON) == LOW && millis() >= prev_press + DEBOUNCE) {
    digitalWrite(GREEN_LED, HIGH);
    prev_press = millis();

    // if pressed for first time, start new recording
    if (!recording) {
      start_record();
      recording = true;
      uint8_t data[] = "RECORD";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      

      
      #ifdef DEBUG
        Serial.println("Started recording");
      #endif
    }
    
    // otherwise record event
    else {
      if (logfile) {
        logfile.print("EVENT ");
        logfile.println(event_count);

        logfile.close();
        logfile = SD.open(fname, FILE_WRITE);

        #ifdef DEBUG
          Serial.print("EVENT ");
          Serial.println(event_count);
        #endif
      }
      event_count++;
    }
    delay(50);
    digitalWrite(GREEN_LED, LOW);
    
  }
}

/*
 * Creates new log and resets count
 */
void start_record() {
  // create new FLIGHTXX.TXT
  for (uint8_t i = 0; i < 100; i++) {
    fname[7] = '0' + i/10;
    fname[8] = '0' + i%10;
    if (!SD.exists(fname)) {
      break;
    }
  }
  logfile = SD.open(fname, FILE_WRITE);
  logfile.println(header);
}
