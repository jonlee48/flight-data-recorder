#include <Stepper.h>

#define DEPLOY_PIN  4  // button to deploy box box n
#define RESET_PIN   7  // button to reset n to NUM_BOXES
#define FORWARD_PIN 5  // button to move belt forward by INTERVAL
#define BACK_PIN    6  // button to move belt back by INTERVAL

#define STEPS 2048     // change this to fit the number of steps per revolution

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(STEPS, 8, 10, 9, 11);

void setup() {
  // set the speed at 10 rpm:
  myStepper.setSpeed(10);
  // initialize the serial port:
  Serial.begin(9600);

  
  // buttons
  pinMode(DEPLOY_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(FORWARD_PIN, INPUT_PULLUP);
  pinMode(BACK_PIN, INPUT_PULLUP);
  
}

void loop() {
  if (digitalRead(FORWARD_PIN) == LOW) {
    moveDegrees(360);
  }

  if (digitalRead(BACK_PIN) == LOW) {
    moveDegrees(-360);
  }

  // step one revolution in the other direction:
  //Serial.println("counterclockwise");
  //myStepper.step(-stepsPerRevolution);
  //delay(500);
}

// positive is clockwise
void moveDegrees(int deg) {
  // convert degrees to steps
  int steps = int(deg/360) * STEPS;
  myStepper.step(steps);
}
