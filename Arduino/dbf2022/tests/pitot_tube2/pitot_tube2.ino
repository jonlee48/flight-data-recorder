// This might be useful for calibration


// File MS4525_Test
//
// A Arduino program to test the operation of pressure sensors in the TE Connectivity MS4525D series (I2C versions only)
//
// Written: 7/9/2019
// Rev.: 1.00
// Changes: First release back to the community. Started with bits of code downloaded from the Arduino forums.
//
// The sensor specific characteristics are setup with constants in the beginning of the program.
// The device page is https://www.te.com/usa-en/product-CAT-BLPS0002.html 4
// The device data sheet can be found at https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+SheetMS4525DOB10pdfEnglishENG_DS_MS4525DO_B10.pdfCAT-BLPS0002 10
//

#include <Wire.h> // Arduino I2C library
#include <stdint.h> // Standard C, Allows explicit data type declaration.

//////////////////////////////////////////////////////////////////////////////////////
// MS4525D sensor characteristics (from the 4/2019 version of the data sheet)
//////////////////////////////////////////////////////////////////////////////////////

// MS4525D sensor I2C address (uncomment the Interface Type of the device you are using)
// Interface Type I
const uint8_t MS4525DAddress = 0x28;
// Interface Type J
//const uint8_t MS4525DAddress = 0x36;
// Interface Type K
//const uint8_t MS4525DAddress = 0x46;
// Interface Type 0
//const uint8_t MS4525DAddress = 0x48;

// MS4525D sensor full scale range and units
const int16_t MS4525FullScaleRange = 1; // 1 psi
//const int16_t MS4525FullScaleRange = 0.0689476; // 1 psi in Bar
//const int16_t MS4525FullScaleRange = 6895; // 1 psi in Pascal
//const int16_t MS4525FullScaleRange = 2; // 2 psi
//const int16_t MS4525FullScaleRange = 5; // 5 psi

// MS4525D Sensor type (A or B) comment out the wrong type assignments
// Type A (10% to 90%)
const int16_t MS4525MinScaleCounts = 1638;
const int16_t MS4525FullScaleCounts = 14746;
// Type B (5% to 95%)
//const int16_t MS4525MinScaleCounts = 819;
//const int16_t MS4525FullScaleCounts = 15563;
const int16_t MS4525Span=MS4525FullScaleCounts-MS4525MinScaleCounts;

//MS4525D sensor pressure style, differential or otherwise. Comment out the wrong one.
//Differential
const int16_t MS4525ZeroCounts=(MS4525MinScaleCounts+MS4525FullScaleCounts)/2;
// Absolute, Gauge
//const int16_t MS4525ZeroCounts=MS4525MinScaleCounts;

//////////////////////////////////////////////////////////////////////////////////////
// end of MS4525D sensor characteristics
//////////////////////////////////////////////////////////////////////////////////////

// fetch_pressure is a function to do the I2C read and extraction of the three data fields
//
byte fetch_pressure(uint16_t &P_dat, uint16_t &T_dat)
{
byte _status;
byte Press_H;
byte Press_L;
byte Temp_H;
byte Temp_L;

Wire.requestFrom(MS4525DAddress, static_cast<uint8_t>(4), static_cast<uint8_t>(true)); //Request 4 bytes, 2 pressure/status and 2 temperature
Press_H = Wire.read();
Press_L = Wire.read();
Temp_H = Wire.read();
Temp_L = Wire.read();

_status = (Press_H >> 6) & 0x03;
Press_H = Press_H & 0x3f;
P_dat = (((uint16_t)Press_H) << 8 ) | Press_L; 
Temp_L = (Temp_L >> 5);
T_dat = (((uint16_t)Temp_H) << 3) | Temp_L;

return _status;

}

// setup is the main function to setup the diagnostic serial port and the I2C port
void setup()
{

Serial.begin(115200);
Wire.begin();
// wait until serial port opens for native USB devices
while (! Serial)
{
delay(1);
}

Serial.println("MS4525DO test");

}

void loop()
{

byte _status; // A two bit field indicating the status of the I2C read
uint16_t P_dat; // 14 bit pressure data
uint16_t T_dat; // 11 bit temperature data
float psi;

_status = fetch_pressure(P_dat, T_dat);

switch (_status)
{
case 0:
//Serial.println("Ok ");
break;
case 1:
Serial.println("Busy");
break;
case 2:
Serial.println("Slate");
break;
default:
Serial.println("Error");
break;
}

psi=(static_cast<int16_t>(static_cast<int16_t>(P_dat)-MS4525ZeroCounts))/static_cast<int16_t>(MS4525Span)*static_cast<int16_t>(MS4525FullScaleRange);
Serial.print("psi:");
Serial.println(psi);

}
