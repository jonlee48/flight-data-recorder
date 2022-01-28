#include <SPI.h>
#include <RH_RF95.h>

// Tunable parameters:
#define POWER 23          // transmitter power from 5 to 23 dBm (default is 13 dBm)


// Probably never change these:
#define BUTTON A5         // Momentary button
#define DEBOUNCE 1500     // Milliseconds between registering presses
#define RED_LED 13        // Built-in LED
#define GREEN_LED 8       // Built-in LED
#define RFM95_CS A1       // LoRa CS pin
#define RFM95_RST A3      // LoRa RST pin
#define RFM95_INT A2      // LoRa INT pin
#define RF95_FREQ 915.0   // LoRa frequency (MHz)

// Instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Global variables (initialized to 0)
unsigned long prev_press;       // Time in millis of last button press


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
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
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
      Serial.print((char*)buf);
      
      Serial.print(",");
      // SAVE A LOG OF RSSI
      Serial.println(rf95.lastRssi(), DEC);
      
      delay(10);
      digitalWrite(RED_LED, LOW);
    }
  }
  // check if record button is pressed
  if (digitalRead(BUTTON) == LOW && millis() >= prev_press + DEBOUNCE) {
    prev_press = millis();
    
    uint8_t data[] = "RECORD";
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
}
