/*  https://www.instructables.com/Radar-System-Using-Ultrasonic-Sensor-Arduino-Uno/
 *   Hello friend Welcome To "Techno-E-Solution"
    Here is a code of "Arduino Radar Using Ultrasonic Sensor"
*/
#include <Servo.h>

const int trigPin = 3;
const int echoPin = 2;

long duration;
int distinCM;

Servo radarServo;

void setup() 
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  radarServo.attach(9);
}
void loop() 
{
  for(int i=0;i<=180;i++)
  {
    radarServo.write(i);
    delay(50);
    
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); 
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distinCM = duration*0.034/2;
    
    Serial.print(i);
    Serial.print("*");
    Serial.print(distinCM);
    Serial.print("#");
  }
  
  for(int i=180;i>=0;i--)
  {
    radarServo.write(i);
    delay(50);
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); 
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distinCM = duration*0.034/2;
    
    Serial.print(i);
    Serial.print("*");
    Serial.print(distinCM);
    Serial.print("#");
  }
}
