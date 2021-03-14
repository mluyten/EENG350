#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

/* Name: Group 6 | EENG350 | Demo 1 Step Response
   Purpose: Step response experiment
   Note: When viewed from the front, left wheel is 1, right wheel is 2. driveModePin is 4 on the motor shield, 5 on the Teensy
*/

int sampleRate = 5;      // 5 milliseconds
int desiredPWM = 128;    //half speed
int radius = 6;          // inches
int wheelbase = 11;      // inches between the center of the two wheels

// Motor Shield Pins -----------------------------------------------------
int driveModePin = 5;    // drive/break mode when HIGH, drive/coast mode when LOW

int m1DirPin = 7;        // sign of voltage
int m1SpeedPin = 9;      // voltage

int m2DirPin = 8;
int m2SpeedPin = 10;

// Calculation variables -------------------------------------------------
int thetaCurrent1 = 0;
int lastThetaCurrent1 = 0;
double angularVelocity1 = 0.0;

int thetaCurrent2 = 0;
int lastThetaCurrent2 = 0;
double angularVelocity2 = 0.0;

double rotationalVelocity = 0.0;
double forwardVelocity = 0.0;

// Encoder setup ---------------------------------------------------------
Encoder myEnc1(1, 2); // wheel 1 (left)
Encoder myEnc2(3, 4); // wheel 2 (right)

void setup() {
  Serial.begin(9600);

  pinMode(driveModePin, OUTPUT);
  digitalWrite(driveModePin, HIGH);

  pinMode(m1DirPin, OUTPUT);
  pinMode(m1SpeedPin, OUTPUT);

  pinMode(m2DirPin, OUTPUT);
  pinMode(m2SpeedPin, OUTPUT);
}

void loop() {
  unsigned long timeBefore = micros();

  make_step_rho(); // Step response forward, replace with make_step_phi() as needed

  thetaCurrent1 = myEnc1.read();
  thetaCurrent2 = -1*myEnc2.read(); // encoder 2 is running in reverse. Switch polarity of counts.

  angularVelocity1 = ((double) (thetaCurrent1 - lastThetaCurrent1)) * 2.0 * 3.14159 / 3200.0 / (0.001 * sampleRate);
  angularVelocity2 = ((double) (thetaCurrent2 - lastThetaCurrent2)) * 2.0 * 3.14159 / 3200.0 / (0.001 * sampleRate);

  rotationalVelocity = ( (double) radius * (angularVelocity1 - angularVelocity2) / wheelbase );
  forwardVelocity = ( (double) radius * (angularVelocity1 + angularVelocity2) / wheelbase );

  lastThetaCurrent1 = thetaCurrent1;
  lastThetaCurrent2 = thetaCurrent2;

  unsigned long timeNow = micros();
  unsigned long timeMain = timeNow - timeBefore; // total time spent doing things in main

  if ( timeMain > (sampleRate * pow(10, 3)) ) { // Error checking to make sure main doesn't take longer than the sampling rate
    Serial.println("Main takes too long.");
  }
  else {
    while ( micros() < timeNow + (sampleRate * pow(10, 3) - timeMain) ); // wait to take a sample exactly every 5ms = 5000us
    Serial.print(timeNow); // current time in micros
    Serial.print("\t");
    Serial.println(forwardVelocity); // sub me with rotationalVelocity as needed
  }
}


void make_step_rho() { // Robot forward step response
  if (millis() < 1000) {
    analogWrite(m1SpeedPin, 0);
    analogWrite(m2SpeedPin, 0);
  } else {
    analogWrite(m1SpeedPin, desiredPWM);
    digitalWrite(m1DirPin, HIGH); // forward left
    analogWrite(m2SpeedPin, desiredPWM);
    digitalWrite(m2DirPin, HIGH); // forward right
  }
}

void make_step_phi() { // Robot rotate counterclockwise step response
  if (millis() < 1000) {
    analogWrite(m1SpeedPin, 0);
    analogWrite(m2SpeedPin, 0);
  } else {
    analogWrite(m1SpeedPin, desiredPWM);
    digitalWrite(m1DirPin, HIGH); // forward left
    analogWrite(m2SpeedPin, desiredPWM);
    digitalWrite(m2DirPin, LOW); // reverse right
  }
}
