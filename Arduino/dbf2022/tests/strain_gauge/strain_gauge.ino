void setup() {
  pinMode(A0, INPUT);
  pinMode(A4, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int v1 = analogRead(A0);
  int v2 = analogRead(A4);
  Serial.print(v1);
  Serial.print(",");
  Serial.println(v2);
  delay(100);
}
