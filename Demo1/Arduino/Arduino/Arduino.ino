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
int wheelbase = 11;      // inches between the center of the two wheels

// Motor Shield Pins -----------------------------------------------------
int driveModePin = 5;    // drive/break mode when HIGH, drive/coast mode when LOW

int m1DirPin = 7;        // sign of voltage
int m1SpeedPin = 9;      // voltage

int m2DirPin = 8;
int m2SpeedPin = 10;


// Distance calculation variables -------------------------------------------------
int thetaCurrent1 = 0;
int lastThetaCurrent1 = 0;
double angularVelocity1 = 0.0;

int thetaCurrent2 = 0;
int lastThetaCurrent2 = 0;
double angularVelocity2 = 0.0;

double rotationalVelocity = 0.0;
double forwardVelocity = 0.0;

// Forward velocity PI controller variables ---------------------------------
float desiredForward = 0.0, deltaForward = 0.0;
float v_bar = 0.0;  //PI Output
//float KpForward = 0.05213617933710021, KiForward = 0.0506808966855011;
float KpForward = 0.0085398, KiForward = 0.055196;
float IForward = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

// Rotational velocity PI controller variables ---------------------------------
float desiredRotational = 0.0, deltaRotational = 0.0;
float delta_v = 0.0;  //PI Output
float KpRotational = 0.014802, KiRotational = 0.13901;
float IRotational = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

// Angle and Position controller variables ----------------------------------
float desiredPosition = 36.0, deltaPosition = 0.0, currentPosition = 0.0;
float desiredAngle = 0.0, deltaAngle = 0.0, currentAngle = 0.0;
float KpPos = 0.5, KpAng = 0.01;

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

  if (lastThetaCurrent1 != thetaCurrent1) {
    angularVelocity1 = ((double) (thetaCurrent1 - lastThetaCurrent1)) * 2.0 * 3.14159 / 3200.0 / (0.000001 * (micros() - lastChange1));
    lastChange1 = micros();
    lastThetaCurrent1 = thetaCurrent1;
    velocityChanged = true;
  }

  if (lastThetaCurrent2 != thetaCurrent2) {
    angularVelocity2 = ((double) (thetaCurrent2 - lastThetaCurrent2)) * 2.0 * 3.14159 / 3200.0 / (0.000001 * (micros() - lastChange2));
    lastChange2 = micros();
    lastThetaCurrent2 = thetaCurrent2;
    velocityChanged = true;
  }

  if (velocityChanged) {
    rotationalVelocity = ( (double) radius * (angularVelocity1 - angularVelocity2) / wheelbase );
    forwardVelocity = ( (double) radius * (angularVelocity1 + angularVelocity2) / 2 );
    velocityChanged = false;
    //Serial.print(deltaPosition);
    //Serial.print("\t");
    //Serial.println(deltaForward);
  }

  currentPosition = (thetaCurrent1 + thetaCurrent2) * 3.14 * 2 * radius / 3200 / 2;

  deltaPosition = desiredPosition - currentPosition;
  deltaAngle = desiredAngle - currentAngle;

  desiredForward = KpPos * abs(deltaPosition);
  desiredAngle = KpAng * abs(deltaAngle);
  
  if (desiredForward > 10)
    desiredForward = 18;

  if (desiredAngle > 5)
    desiredAngle = 5;

  deltaForward = desiredForward - forwardVelocity;
  deltaRotational = desiredRotational - rotationalVelocity;

  IForward = IForward + 0.000001 * sampleRate * deltaForward;
  IRotational = IRotational + 0.000001 * sampleRate * deltaRotational;

  v_bar = KpForward * forwardVelocity + KiForward * IForward;
  delta_v = KpRotational * rotationalVelocity + KiRotational * IRotational;

  v_a1 = (v_bar + delta_v) / 2;
  v_a2 = (v_bar - delta_v) / 2;

  analogWrite(m1SpeedPin, ( (int) (abs(v_a1) * 256) + 8) );
  analogWrite(m2SpeedPin, ( (int) (abs(v_a2) * 256) + 8) );
  if (v_a1 > 0) {
    if (deltaPosition >= 0) 
      digitalWrite(m1DirPin, HIGH);
    else
      digitalWrite(m1DirPin, LOW);
  } else {
    if (deltaPosition >= 0)
      digitalWrite(m1DirPin, LOW);
    else 
      digitalWrite(m1DirPin, HIGH);
  }
  if (v_a2 > 0) {
    if (deltaPosition >= 0) 
      digitalWrite(m2DirPin, HIGH);
    else
      digitalWrite(m2DirPin, LOW);
  } else {
    
    digitalWrite(m2DirPin, LOW);
  }

  timeNow = micros();

  if ( timeNow - timeBefore > sampleRate ) { // Error checking to make sure main doesn't take longer than the sampling rate
    Serial.println("Main takes too long.");
  }
  else {
    while ( micros() < sampleRate + timeBefore ); // wait to take a sample exactly every 5us
  }  
}
