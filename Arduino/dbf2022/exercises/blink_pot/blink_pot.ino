int interval = 1000;
#define LED 9
#define POT A0
void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  pinMode(POT, INPUT);
  Serial.begin(9600);
}
unsigned long lastTime = millis();
bool on = false;
void loop() {
  
  // put your main code here, to run repeatedly:
  int pot = analogRead(POT);
  interval = map(pot,0,1023,0,255);
  //interval = pot;
  analogWrite(LED, interval);
  Serial.println(interval);
  /*
  if (millis() - lastTime > interval && !on) {
    Serial.println("high");
    digitalWrite(12, HIGH);
    lastTime = millis();
    on = true;
  }
  //delay(interval);
  if (millis() - lastTime > interval && on) {
    Serial.println("low");
    digitalWrite(12, LOW);
    lastTime = millis();
    on = false;
  }
  //delay(interval);
  */
}
