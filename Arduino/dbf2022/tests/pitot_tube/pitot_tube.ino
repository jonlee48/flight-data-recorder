/* Jonathan Lee
 * 1/25/22
 * 
 * Reads velocity from pitot tube. Outputs the raw velocity and its moving average
 * 
 * Wiring:
 *  5V to 3.3V output of Adafruit Feather
 *  SDA to SDA (with 10k pullup resistor to 3.3V)
 *  SCL to SCL (with 10k pullup resistor to 3.3V)
 *  Gnd to Gnd
 *  
 * Adapted from
 * https://forum.arduino.cc/t/ms-4525do/298848
 * Post #5, best working solution on the forum
 * 
 * Also take a look at PX4-Autopilot drivers
 * https://github.com/PX4/PX4-Autopilot/blob/master/src/drivers/differential_pressure/ms4525/ms4525_airspeed.cpp
 */
#include <Wire.h>   //I2C library 0x28H 
byte fetch_pressure(unsigned int *p_Pressure); //convert value to byte data type

#define NUM_AVG 20  // how many of the last data points are in the moving average

double moving_avg(double last);
double prev[NUM_AVG];



void setup(void)
{
  Wire.begin();
  delay(500);
  
  Serial.begin(9600);
  while(!Serial);
  //Serial.println("Airspeed (m/s), and moving average (m/s)");
}

void loop()
{
  byte _status;
  unsigned int P_dat;
  unsigned int T_dat;
  double PR;
  double TR;
  double V;
  double VV;
  while (1)
  {
    // fetch_pressure returns sensor status and updates P_dat and T_dat
    _status = fetch_pressure(&P_dat, &T_dat);

    switch (_status)
    {
      case 0: //Serial.println("Ok ");
        break;
      case 1: Serial.println("Busy");
        break;
      case 2: Serial.println("Slate");
        break;
      default: Serial.println("Error");
        break;
    }


    PR = (double)((P_dat-819.15)/(14744.7)) ;
    PR = (PR - 0.49060678) ;
    PR = abs(PR);
     V = ((PR*13789.5144)/1.225);
    VV = (sqrt((V)));

    
    TR = (double)((T_dat*0.09770395701));
    TR = TR-50;
    

    double avg = moving_avg(VV);
    
   //Serial.print("raw Pressure:");
   //Serial.println(P_dat);
   //Serial.println(P_dat,DEC);
   //Serial.println(P_dat,BIN);
   //Serial.print("raw counts:");
   //Serial.print(P_dat);
   Serial.print("pressure psi:");
   Serial.print(PR,10);
   Serial.print(" ");
   //Serial.print("raw Temp:");
   //Serial.println(T_dat);
   //Serial.print("temp:");
   //Serial.println(TR);
   //Serial.print("speed m/s :");
   Serial.print(VV,5);
   Serial.print(", ");
   Serial.println(avg);
   
  
   delay(10);
  }
}

// fetch_pressure returns sensor status and updates P_dat and T_dat
byte fetch_pressure(unsigned int *p_P_dat, unsigned int *p_T_dat)
{


  byte address, Press_H, Press_L, _status;
  unsigned int P_dat;
  unsigned int T_dat;

  address = 0x28;  // I2C address of MS4525 sensor
  Wire.beginTransmission(address);
  Wire.endTransmission();
  delay(10); // <-- CAN WE REDUCE THIS DELAY?

  Wire.requestFrom((int)address, (int) 4);//Request 4 bytes need 4 bytes are read
  Press_H = Wire.read();
  Press_L = Wire.read();
  byte Temp_H = Wire.read();
  byte  Temp_L = Wire.read();
  Wire.endTransmission();


  // Math to obtain data from 14 bit string
  _status = (Press_H >> 6) & 0x03;
  Press_H = Press_H & 0x3f;
  P_dat = (((unsigned int)Press_H) << 8) | Press_L;
  *p_P_dat = P_dat;

  Temp_L = (Temp_L >> 5);
  T_dat = (((unsigned int)Temp_H) << 3) | Temp_L;
  *p_T_dat = T_dat;
  return (_status);
}

// update the moving average by adding a new data point to it
double moving_avg(double last) {

  for (int i = NUM_AVG - 1; i > 0; i--) {
    prev[i] = prev[i-1];
  }
  prev[0] = last;

  double sum = 0.0;
  for (int i = 0; i < NUM_AVG; i++) {
    sum += prev[i];
  }

  return sum/NUM_AVG;
}
