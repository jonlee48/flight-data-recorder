void setup() {
  Serial.begin(9600); 
}

void loop() {
  int y1 = analogRead(A1);


  Serial.println(y1);

  // Remove this delay
  delay(100);
}
/*
byte PWM_PIN = A0;

 
int pwm_value;
 
void setup() {
  pinMode(PWM_PIN, INPUT);
  Serial.begin(9600);
}
 
void loop() {
  pwm_value = pulseIn(PWM_PIN, HIGH);
  Serial.println(pwm_value);
}
*/
