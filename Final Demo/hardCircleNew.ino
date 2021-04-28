#include <Encoder.h>

/* Name: Group 6 | EENG350 | Demo 1 Step Response
   Purpose: Step response experiment
   Note: Left wheel is 1, right wheel is 2. (When facing camera)
*/

int sampleRate = 5;      // 5 milliseconds
int desiredPWM = 128;    //half speed
int radius = 3;          // inches
int wheelbase = 10.71;      // inches between the center of the two wheels

double v1 = 0.0;
double v2 = 0.0;
double pi = 3.14;
int v_max = 12;          // max desired velocity, inches per second, get from robot step response
int rotation_time = 5;   // seconds to complete the circle, arbitrary

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
Encoder myEnc1(1, 2); // left
Encoder myEnc2(3, 4);// right

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

  make_circle();

  thetaCurrent1 = myEnc1.read();
  thetaCurrent2 = myEnc2.read();
  /*
  angularVelocity1 = ((double) (thetaCurrent1 - lastThetaCurrent1)) * 2.0 * 3.14159 / 3200.0 / (0.001 * sampleRate);
  angularVelocity2 = ((double) (thetaCurrent2 - lastThetaCurrent2)) * 2.0 * 3.14159 / 3200.0 / (0.001 * sampleRate);

  rotationalVelocity = ( (double) radius * (angularVelocity1 - angularVelocity2) / wheelbase );
  forwardVelocity = ( (double) radius * (angularVelocity1 + angularVelocity2) / wheelbase );

  lastThetaCurrent1 = thetaCurrent1;
  lastThetaCurrent2 = thetaCurrent2;*/

  unsigned long timeNow = micros();
  unsigned long timeMain = timeNow - timeBefore; // total time spent doing things in main

  if ( timeMain > (sampleRate * pow(10, 3)) ) { // Error checking to make sure main doesn't take longer than the sampling rate
    Serial.println("Main takes too long.");
  }
  else {
    while ( micros() < timeNow + (sampleRate * pow(10, 3) - timeMain) ); // wait to take a sample exactly every 5ms = 5000us
    //Serial.print(timeNow); // current time in micros
    //Serial.print("\t");
    //Serial.println(forwardVelocity); // sub me with rotationalVelocity as needed
  }
}


void make_circle() { // Robot forward step response
  v1 = (2 * pi * (radius + wheelbase) )/ rotation_time;
  v2 = (2 * pi * radius )/ rotation_time;
  
  if (millis() < 1000) {
    analogWrite(m1SpeedPin, 0);
    analogWrite(m2SpeedPin, 0);
  } else if (millis() < 6000){
    analogWrite(m1SpeedPin, (int) (256*v1/v_max) );
    digitalWrite(m1DirPin, HIGH); // forward left
    analogWrite(m2SpeedPin, (int) (256*v2/v_max) );
    digitalWrite(m2DirPin, HIGH); // forward right
  } else {
     analogWrite(m1SpeedPin, 0);
     analogWrite(m2SpeedPin, 0);
  }
}
