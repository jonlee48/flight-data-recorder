/* DBF 2022 SMC Test Program
 *  Deploys a box upon button press, and moves remaining boxes to center of mass.
 *  Two buttons to increment belt forward/backward
 *  Button to reset state back to NUM_BOXES
 *  See wiring diagram on TinkerCAD
 */
 
#include <Stepper.h> // Make sure to import this libary 

 
/* Set the I/O pins here */
//#define SERVO_PIN 
#define DEPLOY_PIN  4  // button to deploy box box n
#define BUILTIN_LED 13 // LED on Arduino, not set to do anything currently

#define INPUT_SIGNAL A0

/* Motor Specs */
#define STEPS       2048  // drive mode
#define MOTOR_PIN1  8     // IN1 on the ULN2003 driver
#define MOTOR_PIN2  9     // IN2 on the ULN2003 driver
#define MOTOR_PIN3  10    // IN3 on the ULN2003 driver
#define MOTOR_PIN4  11    // IN4 on the ULN2003 driver
#define MAX_SPEED   10.0  // just go fast

/* SMC Parameters (all distances in inches) */
#define NUM_BOXES     2    // num boxes that start on belt
#define SERVO_RADIUS  0.375  // radius of the drive wheel
#define BELT_LENGTH   13.75 // distance btwn center of drive wheels
#define BOX_WIDTH     2.5  // width of box on belt
#define DIST_BTWN_BOX 0.0625  // distance between boxes on belt
#define XTRA_PUSH     0.4475  // extra distance to push box from end of belt
#define INTERVAL      0  // when forward/back buttons are pressed, move belt by this amount
//#define THRESHOLD 1500   // PWM value to trigger deployment when connected to receiver

// have to initialize an AccelStepper otherwise i get a segmentation fault
Stepper stepper1(STEPS, MOTOR_PIN1, MOTOR_PIN3, MOTOR_PIN2, MOTOR_PIN4);

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
  // buttons
  pinMode(DEPLOY_PIN, INPUT_PULLUP);
  pinMode(INPUT_SIGNAL, INPUT);

  stepper1.setSpeed(MAX_SPEED);

  // For debugging
  Serial.begin(9600);
}

void moveDegrees(int deg) {
  // convert degrees to steps
  int steps = int(deg/360) * STEPS;
  stepper1.step(steps);
}

void loop(void) {  
  int pwm_value = pulseIn(INPUT_SIGNAL, HIGH);
  if ((digitalRead(DEPLOY_PIN) == LOW && n > 0)||((pwm_value < 1200)&&(pwm_value > 1000))) {
    Serial.print("deploying box: ");
    Serial.println(n);
    // always assumes current position is centered
    
    /* Calculate distance occupied by n boxes, and distance to
     *  from the last box to the end of the belt.
     * Convert that distance to degrees and turn motor by that amount
     */
    double dist_occupied_n = dist_occupied(n);
    double dist_to_end = (BELT_LENGTH/2.0) - (dist_occupied_n/2.0);
    double dist_to_deploy = dist_to_end + XTRA_PUSH
    Serial.print("moving (inches): ");
    Serial.println(dist_to_deploy);
    double deg_to_deploy = dist_to_degrees(dist_to_deploy);
    // turn motor by deg_to_deploy
    moveDegrees(deg_to_deploy);
    

    Serial.println("deploying");
    
    n--;

    /* Return the boxes to center by going backward the same amount minus
     *  the distance of 1 less box
     */
    double diff_occupied = (dist_occupied_n - dist_occupied(n))/2.0;
    double dist_back = diff_occupied - dist_to_deploy;
    Serial.print("moving back (inches): ");
    Serial.println(dist_back);
    double deg_to_center = dist_to_degrees(dist_back);
    // turn motor by deg_to_center
    moveDegrees(deg_to_center);

    Serial.println("returning to position");
    
  }

}
