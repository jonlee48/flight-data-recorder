/*
 * Simple sketch to blink the 6 LEDs on the sensor one at a time
 * 
 */
#define INPUT_SIGNAL A0
// the setup function runs once when you press reset or power the board
void setup() {

  pinMode(5, OUTPUT); // LED
  pinMode(6, OUTPUT); // LED
  pinMode(9, OUTPUT); // LED
  pinMode(10, OUTPUT); // LED
  pinMode(11, OUTPUT); // LED
  pinMode(12, OUTPUT); // LED
  pinMode(INPUT_SIGNAL, INPUT); // LED

}
const int low = 50;
const int high = 20;
const int threshold = 1500; //1700 was effective
// the loop function runs over and over again forever
void loop() {
  
  int pwm_value = pulseIn(INPUT_SIGNAL, HIGH);
  if (pwm_value > threshold && pwm_value < 2000)   {
  digitalWrite(5, HIGH);
  delay(high);
  digitalWrite(5, LOW);
  delay(low);
  digitalWrite(9, HIGH);
  delay(high);
  digitalWrite(9, LOW);
  delay(low);
  digitalWrite(6, HIGH);
  delay(high);
  digitalWrite(6, LOW);
  delay(low);
  digitalWrite(10, HIGH);
  delay(high);
  digitalWrite(10, LOW);
  delay(low);
  digitalWrite(11, HIGH);
  delay(high);
  digitalWrite(11, LOW);
  delay(low);
  digitalWrite(12, HIGH);   
  delay(high);              
  digitalWrite(12, LOW);   
  delay(low);             
  }
  
}
