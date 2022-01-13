#include <Servo.h>

Servo servo1;
Servo servo2;


 
int pwm_value;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  servo1.attach(3);
//  servo2.attach(2);
  //pinMode(2, INPUT);
  //pinMode(3, INPUT);
  pinMode(3, INPUT);
  Serial.begin(115200);
}

void loop() {

  pwm_value = pulseIn(3, HIGH);
  Serial.println(pwm_value);

  
}
