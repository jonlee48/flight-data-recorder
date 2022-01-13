/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04


Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
const int wait = 50;
int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
}

float echo() {
  long duration; // variable for the duration of sound wave travel
  float distance; // variable for the distance measurement
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 * 0.5; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  return distance;
}

void loop() {
  for (pos = 15; pos <= 165; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(wait);                       // waits 15ms for the servo to reach the position
    Serial.println(String(float(echo())) + "," + String(float(pos)) + "," + String(float(millis()/1000)));
    delay(wait);
  }
  for (pos = 165; pos >= 15; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(wait);                       // waits 15ms for the servo to reach the position
    Serial.println(String(float(echo())) + "," + String(float(pos)) + "," + String(float(millis()/1000)));
    delay(wait);
  }
}
