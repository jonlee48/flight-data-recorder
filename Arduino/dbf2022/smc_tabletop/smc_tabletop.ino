/* DBF 2022 SMC Test Program
 *  Deploys a box upon button press, and moves remaining boxes to center of mass.
 *  Two buttons to increment belt forward/backward
 *  Button to reset state back to NUM_BOXES
 *  See wiring diagram on TinkerCAD
 */
 
#include <AccelStepper.h> // Make sure to import this libary 

 
/* Set the I/O pins here */
//#define SERVO_PIN 
#define DEPLOY_PIN  4  // button to deploy box box n
#define RESET_PIN   7  // button to reset n to NUM_BOXES
#define FORWARD_PIN 5  // button to move belt forward by INTERVAL
#define BACK_PIN    6  // button to move belt back by INTERVAL
#define BUILTIN_LED 13 // LED on Arduino, not set to do anything currently

/* Motor Specs */
#define STEPS       2048  // drive mode
#define MOTOR_PIN1  8     // IN1 on the ULN2003 driver
#define MOTOR_PIN2  9     // IN2 on the ULN2003 driver
#define MOTOR_PIN3  10    // IN3 on the ULN2003 driver
#define MOTOR_PIN4  11    // IN4 on the ULN2003 driver
#define ACCEL       100000.0  // set to practically instantaneous accel
#define MAX_SPEED   15.0  // just go fast

/* SMC Parameters (all distances in inches) */
#define NUM_BOXES     6    // num boxes that start on belt
#define SERVO_RADIUS  1.0  // radius of the drive wheel
#define BELT_LENGTH   24.0 // distance btwn center of drive wheels
#define BOX_WIDTH     3.0  // width of box on belt
#define DIST_BTWN_BOX 1.0  // distance between boxes on belt
#define XTRA_PUSH     0.0  // extra distance to push box from end of belt
#define INTERVAL      1.0  // when forward/back buttons are pressed, move belt by this amount
//#define THRESHOLD 1500   // PWM value to trigger deployment when connected to receiver

// have to initialize an AccelStepper otherwise i get a segmentation fault
AccelStepper stepper1(STEPS, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);

/* Global Vars */
int n = NUM_BOXES;

// all helper function calculations measured from **center** of boxes

// return distance occupied by n boxes
double dist_occupied(int n) {
  if (n < 1)
    return 0.0;

  int num_spaces = n - 1;
  double total_dist = num_spaces *(DIST_BTWN_BOX + BOX_WIDTH);
  return total_dist;
}

// return degrees of servo rotation corresonding to a belt travel distance
double dist_to_degrees(double dist) {
  double rotations = dist / (2.0 * 3.1415 * SERVO_RADIUS);
  double deg = rotations * 360.0;
  return deg;
}


void setup(void)
{
  // onboard LED
  pinMode(BUILTIN_LED, OUTPUT);

  // buttons
  pinMode(DEPLOY_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(FORWARD_PIN, INPUT_PULLUP);
  pinMode(BACK_PIN, INPUT_PULLUP);

  // Motor
  stepper1.setMaxSpeed(MAX_SPEED);
  stepper1.setSpeed(MAX_SPEED);
  stepper1.setAcceleration(ACCEL);  

  // For debugging
  Serial.begin(9600);
}


void loop(void) {  
  // Uncomment these lines if using reciever to control belt
  //int pwm_value = pulseIn(DEPLOY_PIN, HIGH);
  //if (pwm_value > threshold && pwm_value < 2000){

  if (digitalRead(DEPLOY_PIN) == LOW && n > 0) {
    Serial.print("deploying box: ");
    Serial.print(n);
    // always assumes current position is centered
    
    /* Calculate distance occupied by n boxes, and distance to
     *  from the last box to the end of the belt.
     * Convert that distance to degrees and turn motor by that amount
     */
    double dist_occupied_n = dist_occupied(n);
    double dist_to_end = (BELT_LENGTH/2.0) - (dist_occupied_n/2.0);
    double deg_to_deploy = dist_to_degrees(dist_to_end + XTRA_PUSH);
    // turn motor by deg_to_deploy
    stepper1.move(deg_to_deploy);
    stepper1.runToPosition();

    Serial.println("deploying");
    
    n--;

    /* Return the boxes to center by going backward the same amount minus
     *  the distance of 1 less box
     */
    double diff_occupied = (dist_occupied_n - dist_occupied(n))/2.0;
    double deg_to_center = dist_to_degrees(diff_occupied) - deg_to_deploy;
    // turn motor by deg_to_center
    stepper1.move(deg_to_center);
    stepper1.runToPosition();

    Serial.println("returning to position");
    
  }

  if (digitalRead(FORWARD_PIN) == LOW) {
    double deg = dist_to_degrees(INTERVAL);
    // turn motor by deg
    stepper1.move(deg);
    stepper1.runToPosition();
    
  }

  if (digitalRead(BACK_PIN) == LOW) {
    double deg = -1.0 * dist_to_degrees(INTERVAL);
    // turn motor by deg
    stepper1.move(deg);
    stepper1.runToPosition();
    
  }

  if (digitalRead(RESET_PIN) == LOW) {
    n = NUM_BOXES; 
  }

}
