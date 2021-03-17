/* Old Code
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

 Name: Group 6 | EENG350 | Demo 1 Step Response
   Purpose: Step response experiment
   Note: when viewed from the front, left wheel is 1, right wheel is 2. driveModePin is 4 on the motor shield, 5 on the Teensy


// Test Sampling Method --------------------------------------------
unsigned long lastChange1 = 0;
unsigned long lastChange2 = 0;
bool velocityChanged = false;

int sampleRate = 5;      // 5 us
int radius = 3;          // inches
int wheelbase = 11;      // inches between the center of the two wheels

// Motor Shield Pins -----------------------------------------------------
int driveModePin = 5;    // drive/break mode when HIGH, drive/coast mode when LOW

int m1DirPin = 7;        // sign of voltage
int m1SpeedPin = 9;      // voltage

int m2DirPin = 8;
int m2SpeedPin = 10;

int a1SpeedPin = 0;
int a2SpeedPin = 0;
int a1DirPin = 0;
int a2DirPin = 0;

// Distance calculation variables -------------------------------------------------
int thetaCurrent1 = 0;
int deltaTheta1 = 0;
int lastThetaCurrent1 = 0;
double angularVelocity1 = 0.0;

int thetaCurrent2 = 0;
int deltaTheta2 = 0;
int lastThetaCurrent2 = 0;
double angularVelocity2 = 0.0;

double rotationalVelocity = 0.0;
double forwardVelocity = 0.0;

// Forward velocity PI controller variables ---------------------------------
float desiredForward = 0.0, deltaForward = 0.0;
float v_bar = 0.0;  //PI Output
//float KpForward = 0.05213617933710021, KiForward = 0.0506808966855011;
float KpForward = 0.0185398, KiForward = 0.055196;
float IForward = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

// Rotational velocity PI controller variables ---------------------------------
float desiredRotational = 0.0, deltaRotational = 0.0;
float delta_v = 0.0;  //PI Output
float KpRotational = 0.144802, KiRotational = 0.05901;
float IRotational = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

// Angle and Position controller variables ----------------------------------
float desiredPosition = -24, deltaPosition = 0.0, currentPosition = 0.0;
float desiredAngle = 0.0, deltaAngle = 0.0, currentAngle = 0.0;
float KpPos = 0.5, KpAng = 1;

// Motor Voltage Calcs-----------------------------------------------------
double v_a1 = (v_bar + delta_v) / 2;
double v_a2 = (v_bar - delta_v) / 2;

unsigned long timeBefore = micros();
unsigned long timeNow = micros();

// Encoder setup ---------------------------------------------------------
Encoder myEnc1(1, 2); // wheel 1 (left)
Encoder myEnc2(3, 4); // wheel 2 (right)

void setup() {
  Serial.begin(115200);

  pinMode(driveModePin, OUTPUT);
  digitalWrite(driveModePin, HIGH);

  pinMode(m1DirPin, OUTPUT);
  pinMode(m1SpeedPin, OUTPUT);

  pinMode(m2DirPin, OUTPUT);
  pinMode(m2SpeedPin, OUTPUT);
}

void loop() {
  timeBefore = micros();

  thetaCurrent1 = -1 * myEnc1.read(); //Switch polarity of counts.
  thetaCurrent2 = myEnc2.read();

  if (lastThetaCurrent1 != thetaCurrent1) { // Updates wheel 1 variables whenever wheel 1 encoder counts changes
    deltaTheta1 = (thetaCurrent1 - lastThetaCurrent1); // Change in encoder counts
    angularVelocity1 = (double) deltaTheta1 * 2.0 * 3.14159 / 3200.0 / (0.000001 * (micros() - lastChange1)); // Wheel 1 angular velocity in rad/s
    lastChange1 = micros(); // Updates time when the wheel 1 variables were updated
    lastThetaCurrent1 = thetaCurrent1; // Updates current encoder counts
    velocityChanged = true; // Enters velocity calculation if statement
  }

  if (lastThetaCurrent2 != thetaCurrent2) { // Updates wheel 2 variables whenever wheel 1 encoder counts changes
    deltaTheta2 = (thetaCurrent2 - lastThetaCurrent2); // Change in encoder counts
    angularVelocity2 = (double) deltaTheta2 * 2.0 * 3.14159 / 3200.0 / (0.000001 * (micros() - lastChange2)); // Wheel 2 angular velocity in rad/s
    lastChange2 = micros(); // Updates time when the wheel 1 variables were updated
    lastThetaCurrent2 = thetaCurrent2; // Updates current encoder counts
    velocityChanged = true; // Enters velocity calculation if statement
  }

  if (velocityChanged) { // If encoder counts changes on either wheel, update velocity variables
    rotationalVelocity = ( (double) radius * (angularVelocity1 - angularVelocity2) / wheelbase ); // Robot's rotational velocity in rad/s
    forwardVelocity = ( (double) radius * (angularVelocity1 + angularVelocity2) / 2 ); // Robot's forward velocity in in/s
    velocityChanged = false; // ensure this loop does not execute unless encoder counts changes
    currentAngle = currentAngle + ( (double) radius * (deltaTheta1 - deltaTheta2) * 2.0 * 3.14159 / 3200.0 / wheelbase); // Updates the current angle of the robot in rad
    Serial.print(v_bar);
    Serial.print("\t");
    Serial.print(delta_v);
    Serial.print("\t");
    Serial.print(currentPosition);
    Serial.print("\t");
    Serial.print(deltaPosition);
    Serial.print("\t");
    Serial.print(deltaAngle);
    Serial.print("\t");
    Serial.println(deltaRotational);
  }

  currentPosition = (thetaCurrent1 + thetaCurrent2) * 3.14 * radius / 3200; // current position of the robot in inches
  deltaPosition = desiredPosition - currentPosition; // Position error ie. distance (in inches) from desired position
  desiredForward = KpPos * abs(deltaPosition); // Forward velocity P controller. Takes difference in position and multiplies it by a proportional controller to get desired velocity
  if (desiredForward > 10) // Saturation for position P controller. Ensures that desired forward velocity cannot be greater than physically possible
    desiredForward = 18;

  deltaAngle = desiredAngle - currentAngle; // Angle error ie. distance (in radians) from desired angle
  desiredRotational = KpAng * abs(deltaAngle); // Rotational velocity P controller. Takes difference in angle and multiplies it by a proportional controller to get desired velocity
  if (desiredRotational > 5) // Saturation for angle P controller. Ensures that desired rotational velocity cannot be greater than physically possible
    desiredRotational = 5;

  deltaForward = desiredForward - abs(forwardVelocity); // Forward velocity error ie difference (in in/s) between desired and actual forward velocity
  IForward = IForward + 0.000001 * sampleRate * deltaForward; // Integral of the forward velocity PI controller (in in)
  v_bar = KpForward * deltaForward + KiForward * IForward; // V_bar voltage from forward velocity PI controller
  
  deltaRotational = desiredRotational - abs(rotationalVelocity); // Rotational velocity error ie difference (in in/s) between desired and actual forward velocity
  IRotational = IRotational + 0.000001 * sampleRate * deltaRotational; // Integral of the rotational velocity PI controller (in in)
  delta_v = KpRotational * deltaRotational + KiRotational * IRotational; // delta_v voltage from forward velocity PI controller

  v_a1 = (v_bar + delta_v) / 2; 
  v_a2 = (v_bar - delta_v) / 2;

  
  if (deltaAngle > 0) { 
    a1SpeedPin = m1SpeedPin;
    a2SpeedPin = m2SpeedPin;
    a1DirPin = m1DirPin;
    a2DirPin = m2DirPin;
  }
  else {
    a1SpeedPin = m2SpeedPin;
    a2SpeedPin = m1SpeedPin;
    a1DirPin = m2DirPin;
    a2DirPin = m1DirPin;
  }

  analogWrite(a1SpeedPin, ( (int) (abs(v_a1) * 256)) );
  analogWrite(a2SpeedPin, ( (int) (abs(v_a2) * 256)) );

  if (v_a1 > 0 && v_a2 > 0) {
    if (deltaPosition >= 0) {
      digitalWrite(a1DirPin, HIGH);
      digitalWrite(a2DirPin, HIGH);
    }
    else {
      digitalWrite(a1DirPin, LOW);
      digitalWrite(a2DirPin, LOW);
    }
  } else if (v_a1 > 0 && v_a2 < 0) {
    digitalWrite(a1DirPin, HIGH);
    digitalWrite(a2DirPin, LOW);

  } else if (v_a1 < 0 && v_a2 > 0) {
    digitalWrite(a1DirPin, LOW);
    digitalWrite(a2DirPin, HIGH);
  } else {
    if (deltaPosition >= 0) {
      digitalWrite(a1DirPin, LOW);
      digitalWrite(a2DirPin, LOW);

    }
    else {
      digitalWrite(a1DirPin, LOW);
      digitalWrite(a2DirPin, LOW);
    }
  }

  timeNow = micros();

  if ( timeNow - timeBefore > sampleRate ) { // Error checking to make sure main doesn't take longer than the sampling rate
  }
  else {
    while ( micros() < sampleRate + timeBefore ); // wait to take a sample exactly every 5us
  }  
} */

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

/* Name: Group 6 | EENG350 | Demo 1 Step Response
   Purpose: Step response experiment
   Note: when viewed from the front, left wheel is 1, right wheel is 2. driveModePin is 4 on the motor shield, 5 on the Teensy
*/

// Test Sampling Method --------------------------------------------
unsigned long lastChange1 = 0;
unsigned long lastChange2 = 0;
bool velocityChanged = false;

int sampleRate = 5;      // 5 us
int radius = 3;          // inches
double wheelbase = 11.1;      // inches between the center of the two wheels

// Motor Shield Pins -----------------------------------------------------
int driveModePin = 5;    // drive/break mode when HIGH, drive/coast mode when LOW

int m1DirPin = 7;        // sign of voltage
int m1SpeedPin = 9;      // voltage

int m2DirPin = 8;
int m2SpeedPin = 10;

// Distance calculation variables -------------------------------------------------
int thetaCurrent1 = 0;
int deltaTheta1 = 0;
int lastThetaCurrent1 = 0;
double angularVelocity1 = 0.0;

int thetaCurrent2 = 0;
int deltaTheta2 = 0;
int lastThetaCurrent2 = 0;
double angularVelocity2 = 0.0;

double rotationalVelocity = 0.0;
double forwardVelocity = 0.0;

// Forward velocity PI controller variables ---------------------------------
float desiredForward = 0.0, deltaForward = 0.0;
float v_bar = 0.0;  //PI Output
//float KpForward = 0.05213617933710021, KiForward = 0.0506808966855011;
float KpForward = 0.0185398, KiForward = 0.155196;
float IForward = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

// Rotational velocity PI controller variables ---------------------------------
float desiredRotational = 0.0, deltaRotational = 0.0;
float delta_v = 0.0;  //PI Output
float KpRotational = 0.014802, KiRotational = 0.13901;
float IRotational = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

// Angle and Position controller variables ----------------------------------
float desiredPosition = 0.0, deltaPosition = 0.0, currentPosition = 0.0;
float desiredAngle = 3.14/2, deltaAngle = 0.0, currentAngle = 0.0;
float KpPos = 0.75, KpAng = 1.25;

// Motor Voltage Calcs-----------------------------------------------------
double v_a1 = (v_bar + delta_v) / 2;
double v_a2 = (v_bar - delta_v) / 2;

unsigned long timeBefore = micros();
unsigned long timeNow = micros();

// Encoder setup ---------------------------------------------------------
Encoder myEnc1(1, 2); // wheel 1 (left)
Encoder myEnc2(3, 4); // wheel 2 (right)

void setup() {
  Serial.begin(115200);

  pinMode(driveModePin, OUTPUT);
  digitalWrite(driveModePin, HIGH);

  pinMode(m1DirPin, OUTPUT);
  pinMode(m1SpeedPin, OUTPUT);

  pinMode(m2DirPin, OUTPUT);
  pinMode(m2SpeedPin, OUTPUT);

  
  myEnc1.write(0);
  myEnc1.write(0);
}

void loop() {
  timeBefore = micros();

  thetaCurrent1 = -1 * myEnc1.read(); //Switch polarity of counts.
  thetaCurrent2 = myEnc2.read();

  if (lastThetaCurrent1 != thetaCurrent1) { // Updates wheel 1 variables whenever wheel 1 encoder counts changes
    deltaTheta1 = (thetaCurrent1 - lastThetaCurrent1); // Change in encoder counts
    angularVelocity1 = (double) deltaTheta1 * 2.0 * 3.14159 / 3200.0 / (0.000001 * (micros() - lastChange1)); // Wheel 1 angular velocity in rad/s
    lastChange1 = micros(); // Updates time when the wheel 1 variables were updated
    lastThetaCurrent1 = thetaCurrent1; // Updates current encoder counts
    velocityChanged = true; // Enters velocity calculation if statement
  }

  if (lastThetaCurrent2 != thetaCurrent2) { // Updates wheel 2 variables whenever wheel 1 encoder counts changes
    deltaTheta2 = (thetaCurrent2 - lastThetaCurrent2); // Change in encoder counts
    angularVelocity2 = (double) deltaTheta2 * 2.0 * 3.14159 / 3200.0 / (0.000001 * (micros() - lastChange2)); // Wheel 2 angular velocity in rad/s
    lastChange2 = micros(); // Updates time when the wheel 1 variables were updated
    lastThetaCurrent2 = thetaCurrent2; // Updates current encoder counts
    velocityChanged = true; // Enters velocity calculation if statement
  }

  if (velocityChanged) { // If encoder counts changes on either wheel, update velocity variables
    rotationalVelocity = ( (double) radius * (angularVelocity1 - angularVelocity2) / wheelbase ); // Robot's rotational velocity in rad/s
    forwardVelocity = ( (double) radius * (angularVelocity1 + angularVelocity2) / 2 ); // Robot's forward velocity in in/s
    velocityChanged = false; // ensure this loop does not execute unless encoder counts changes
    currentAngle = currentAngle + ( (double) radius * (deltaTheta1 - deltaTheta2) * 3.14159 / 3200.0 / wheelbase); // Updates the current angle of the robot in rad
  }

  currentPosition = (thetaCurrent1 + thetaCurrent2) * 3.14 * radius / 3200; // current position of the robot in inches
  deltaPosition = desiredPosition - currentPosition; // Position error ie. distance (in inches) from desired position
  desiredForward = KpPos * (deltaPosition); // Forward velocity P controller. Takes difference in position and multiplies it by a proportional controller to get desired velocity
  if (desiredForward > 10) // Saturation for position P controller. Ensures that desired forward velocity cannot be greater than physically possible
    desiredForward = 18;

  deltaAngle = desiredAngle - currentAngle; // Angle error ie. distance (in radians) from desired angle
  desiredRotational = KpAng * (deltaAngle); // Rotational velocity P controller. Takes difference in angle and multiplies it by a proportional controller to get desired velocity
  if (desiredRotational > 5) // Saturation for angle P controller. Ensures that desired rotational velocity cannot be greater than physically possible
    desiredRotational = 5;

  deltaForward = desiredForward - forwardVelocity; // Forward velocity error ie difference (in in/s) between desired and actual forward velocity
  if (deltaForward < 0) {
    IForward = IForward + 0.000001 * sampleRate * deltaForward * -1; // Integral of the forward velocity PI controller (in in)
    v_bar = -1 * (KpForward * deltaForward * -1 + KiForward * IForward); // V_bar voltage from forward velocity PI controller
  } else {
    IForward = IForward + 0.000001 * sampleRate * deltaForward; // Integral of the forward velocity PI controller (in in)
    v_bar = KpForward * deltaForward + KiForward * IForward; // V_bar voltage from forward velocity PI controller
  }
  
  deltaRotational = desiredRotational - rotationalVelocity; // Rotational velocity error ie difference (in in/s) between desired and actual forward velocity
  if (deltaRotational < 0) {
    IRotational = IRotational + 0.000001 * sampleRate * deltaRotational * -1; // Integral of the rotational velocity PI controller (in in)
    delta_v = -1 * (KpRotational * deltaRotational * -1 + KiRotational * IRotational); // delta_v voltage from forward velocity PI controller
  } else {
    IRotational = IRotational + 0.000001 * sampleRate * deltaRotational; // Integral of the rotational velocity PI controller (in in)
    delta_v = KpRotational * deltaRotational + KiRotational * IRotational; // delta_v voltage from forward velocity PI controller
  
  }
  
  v_a1 = (v_bar + delta_v) / 2; 
  v_a2 = (v_bar - delta_v) / 2;

  analogWrite(m1SpeedPin, ( (int) (abs(v_a1) * 256)) + 8);
  analogWrite(m2SpeedPin, ( (int) (abs(v_a2) * 256)) + 8 );

  if (v_a1 >= 0) {
    digitalWrite(m1DirPin, HIGH);
  } else {
    digitalWrite(m1DirPin, LOW);
  }
  
  if (v_a2 >= 0) {
    digitalWrite(m2DirPin, HIGH);
  } else {
    digitalWrite(m2DirPin, LOW);
  }

  timeNow = micros();

  if ( timeNow - timeBefore > sampleRate ) { // Error checking to make sure main doesn't take longer than the sampling rate
  }
  else {
    while ( micros() < sampleRate + timeBefore ); // wait to take a sample exactly every 5us
  }  
}
