#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

/* Name: Group 6 | EENG350 | Demo 1 Step Response
   Purpose: Step response experiment
   Note: when viewed from the front, left wheel is 1, right wheel is 2. driveModePin is 4 on the motor shield, 5 on the Teensy
*/

double stepResponse[2][10000];
int sampleIndex = 0;

// Test Sampling Method --------------------------------------------
int nextSample = 0;
unsigned long lastChange1 = 0;
unsigned long lastChange2 = 0;
bool velocityChanged = false;

int sampleRate = 500;      // 0.5 milliseconds
int radius = 3;          // inches
int wheelbase = 11;      // inches between the center of the two wheels

// Motor Shield Pins -----------------------------------------------------
int driveModePin = 5;    // drive/break mode when HIGH, drive/coast mode when LOW

int m1DirPin = 7;        // sign of voltage
int m1SpeedPin = 9;      // voltage

int m2DirPin = 8;
int m2SpeedPin = 10;

// Motor Voltage Calcs-----------------------------------------------------
double v_bar = 0.0;
double delta_v = 1.0;
double v_a1 = (v_bar + delta_v) / 2;
double v_a2 = (v_bar - delta_v) / 2;

// Distance calculation variables -------------------------------------------------
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
  Serial.begin(115200);

  pinMode(driveModePin, OUTPUT);
  digitalWrite(driveModePin, HIGH);

  pinMode(m1DirPin, OUTPUT);
  pinMode(m1SpeedPin, OUTPUT);

  pinMode(m2DirPin, OUTPUT);
  pinMode(m2SpeedPin, OUTPUT);
}

void loop() {
  unsigned long timeBefore = micros();

  make_step(); // Step response function, change v_bar and delta_v as needed

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

    stepResponse[0][sampleIndex] = forwardVelocity;
    stepResponse[1][sampleIndex] = micros();
    sampleIndex++;
  }


  /*
  thetaCurrent1 = -1 * myEnc1.read(); //Switch polarity of counts.
  thetaCurrent2 = myEnc2.read();

  angularVelocity1 = ((double) (thetaCurrent1 - lastThetaCurrent1)) * 2.0 * 3.14159 / 3200.0 / (0.000001 * sampleRate);
  angularVelocity2 = ((double) (thetaCurrent2 - lastThetaCurrent2)) * 2.0 * 3.14159 / 3200.0 / (0.000001 * sampleRate);

  rotationalVelocity = ( (double) radius * (angularVelocity1 - angularVelocity2) / wheelbase );
  forwardVelocity = ( (double) radius * (angularVelocity1 + angularVelocity2) / 2 );

  lastThetaCurrent1 = thetaCurrent1;
  lastThetaCurrent2 = thetaCurrent2;

  unsigned long timeNow = micros();
  unsigned long timeMain = timeNow - timeBefore; // total time spent doing things in main

  if ( timeMain > sampleRate ) { // Error checking to make sure main doesn't take longer than the sampling rate
    //Serial.println("Main takes too long.");
  }
  else {
    while ( micros() < timeNow + (sampleRate - timeMain) ); // wait to take a sample exactly every 5ms = 5000us
    //Serial.print(timeNow); // current time in micros
    //Serial.print("\t");
    //Serial.print(forwardVelocity); // sub me with rotationalVelocity as needed
    //Serial.print("\t");
    //Serial.println(rotationalVelocity);
    stepResponse[0][sampleIndex] = rotationalVelocity;
    stepResponse[1][sampleIndex] = timeNow;
    sampleIndex++;
  }
  */
}


void make_step() { // Robot forward step response
  if (millis() < 1000) {
    analogWrite(m1SpeedPin, 0);
    analogWrite(m2SpeedPin, 0);
  } else if ( millis() >= 1000 && millis() < 2000) {
    analogWrite(m1SpeedPin, ( (int) (abs(v_a1) * 256) ) );
    analogWrite(m2SpeedPin, ( (int) (abs(v_a2) * 256) ) );
    if (v_a1 > 0) {
      digitalWrite(m1DirPin, HIGH);
    } else {
      digitalWrite(m1DirPin, LOW);
    }
    if (v_a2 > 0) {
      digitalWrite(m2DirPin, HIGH);
    } else {
      digitalWrite(m2DirPin, LOW);
    }
  } else {
    analogWrite(m1SpeedPin, 0);
    analogWrite(m2SpeedPin, 0);
    for (int i = 0; i < sampleIndex; i ++) {
      Serial.print(stepResponse[1][i]);
      Serial.print("\t");
      Serial.println(stepResponse[0][i]);
    }
    while (1){};
  }

}
