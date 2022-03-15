
#include <Stepper.h>

#define FORWARD_PIN 3  // button to move belt forward by INTERVAL
#define BACK_PIN    5 // button to move belt back by INTERVAL

#define STEPS 2048     // change this to fit the number of steps per revolution

#define INPUT_SIGNAL A0

// initialize the stepper library on pins 8 through 11:
Stepper myStepper1(STEPS, 8, 10, 9, 11);
//Stepper myStepper2(STEPS, 7, 4, 6, 2);


void setup() {
  // set the speed at 10 rpm:
  myStepper1.setSpeed(10);
  //myStepper2.setSpeed(10);
  // initialize the serial port:
  Serial.begin(9600);

  
  // buttons
  pinMode(BACK_PIN, INPUT_PULLUP);
  pinMode(FORWARD_PIN, INPUT_PULLUP);

  // 
  pinMode(INPUT_SIGNAL, INPUT);
  
}


//const int threshold = 1200;

void loop() {
  int pwm_value = pulseIn(INPUT_SIGNAL, HIGH);
  Serial.println(pwm_value);
  if ((pwm_value < 1200)&&(pwm_value > 1000)) {
    moveDegrees(360);
  }

  //if (digitalRead(BACK_PIN) == LOW) {
    //moveDegrees(-360);
    //Serial.println("pushed");
  //}
}

// positive is clockwise
void moveDegrees(int deg) {
  // convert degrees to steps
  int steps = int(deg/360) * STEPS;
  myStepper1.step(steps);
  
  //for (int i = 0; i < steps; i++) {
    //myStepper1.step(1);
    //myStepper2.step(-1);
  //}
}

