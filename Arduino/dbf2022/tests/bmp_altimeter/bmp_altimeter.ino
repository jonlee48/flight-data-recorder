/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP388 Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>


#define SEALEVELPRESSURE_HPA 1026.753//(1013.25)

Adafruit_BMP3XX bmp; // I2C

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  // See Filter Selection, page 16
  // https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP388-DS001.pdf
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X); // 0.0006 *C resolution
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);  // 0.66 Pa resolution
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  //Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  //Serial.println(" *C");

  Serial.print(", ");
  //Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  //Serial.println(" hPa");

  //Serial.print("Approx. Altitude = ");
  //Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  //Serial.println(" m");

  Serial.println();
  delay(100);
}
