#include <Adafruit_BNO055.h>

#define BNO_RST     A3      // pin number of BNO055 RST
#define BUTTON_TARE 8      // pin number of TARE button, button connects to ground
#define BAUDRATE    115200  // serial port baud rate
#define PERIOD_MS   100      // time between measurements, milliseconds

Adafruit_BNO055 bno = Adafruit_BNO055(1);
unsigned long tprev;

void setup(void)
{
  Serial.begin(BAUDRATE);

  pinMode(BUTTON_TARE, INPUT_PULLUP);

  // initialize BNO055, more reliable than Adafruit's begin()
  Wire.begin();
  Wire.setClock(100000L);           // I2C speed
  digitalWrite(BNO_RST,0);
  pinMode(BNO_RST, OUTPUT);         // assert BNO RST
  delay(2);                         // 1 ms is ample but some microcontrollers truncate the first ms
  pinMode(BNO_RST, INPUT_PULLUP);   // deassert BNO RST
  delay(800);                       // allow time for BNO to boot
  #if 1    // my BNO is mounted with its dot rearward/right/up so choose P3 (see BNO055 datasheet)
    bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);
    bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);
  #endif
  bno.setMode(bno.OPERATION_MODE_NDOF);
  delay(10);                        // allow time for BNO to switch modes

  tprev = millis();
}

void loop(void)
{
  imu::Quaternion q = bno.getQuat();
  // flip BNO/Adafruit quaternion axes to aerospace: x forward, y right, z down
  float temp = q.x();  q.x() = q.y();  q.y() = temp;  q.z() = -q.z();

  static imu::Quaternion tare = {1,0,0,0};
  if (!digitalRead(BUTTON_TARE))
    tare = q.conjugate();
  #if 0   // North-referenced: attach sensor to aircraft any which way, fly level towards magnetic north, click button
    q = q * tare;
  #else   // Screen-referenced: aim sensor towards your screen, click button
    q = tare * q;
  #endif
  q.normalize();

  // convert aerospace quaternion to aerospace Euler, because BNO055 Euler data is broken
  float heading = 180/M_PI * atan2(q.x()*q.y() + q.w()*q.z(), 0.5 - q.y()*q.y() - q.z()*q.z());
  float pitch   = 180/M_PI * asin(-2.0 * (q.x()*q.z() - q.w()*q.y()));
  float roll    = 180/M_PI * atan2(q.w()*q.x() + q.y()*q.z(), 0.5 - q.x()*q.x() - q.y()*q.y());
  heading = heading < 0 ? heading+360 : heading;  // wrap heading to 0..360 convention

  #if 0    // send quaternion
    Serial.print(F("Quaternion: "));
    Serial.print(q.w(), 4);
    Serial.print(F(" "));
    Serial.print(q.x(), 4);
    Serial.print(F(" "));
    Serial.print(q.y(), 4);
    Serial.print(F(" "));
    Serial.print(q.z(), 4);
    Serial.println(F(""));
  #endif
  #if 1    // send Euler angles, my preferred format
    //Serial.print(F("HeadingPitchRoll: "));
    if (heading == heading) { // eliminates NaN readings
      Serial.print(int(heading));  // heading, nose-right is positive
      //Serial.print(F(" "));
      //Serial.print(pitch);    // pitch, nose-up is positive
      //Serial.print(F(" "));
      //Serial.print(roll);     // roll, leftwing-up is positive
      Serial.println(F(""));
    }
  #endif
  #if 0    // send Euler angles, alternate (Adafruit?) format
    Serial.print(F("Orientation: "));
    Serial.print(heading);  // heading, nose-right is positive
    Serial.print(F(" "));
    Serial.print(roll);     // roll, leftwing-up is positive
    Serial.print(F(" "));
    Serial.print(pitch);    // pitch, nose-up is positive
    Serial.println(F(""));
  #endif
  #if 0    // send calibration status
    uint8_t sys, gyro, accel, mag = 0;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print(F("Calibration: "));
    Serial.print(sys, DEC);
    Serial.print(F(" "));
    Serial.print(gyro, DEC);
    Serial.print(F(" "));
    Serial.print(accel, DEC);
    Serial.print(F(" "));
    Serial.println(mag, DEC);
  #endif

  while (millis() - tprev < PERIOD_MS);  // wait until next measurement
  tprev += PERIOD_MS;
}
