#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>

/* Name: Group 6 | EENG350 | Demo 2 Arduino Code

   Purpose: Drive in a circle
   
   Note: when viewed from the front, left wheel is 1, right wheel is 2. driveModePin is 4 on the motor shield, 5 on the Teensy
*/

// Test Sampling Method --------------------------------------------
unsigned long lastChange1 = 0;
unsigned long lastChange2 = 0;
bool velocityChanged = false;

int sampleRate = 5;            // 5 us
double radius = 2.95;           // inches
double wheelbase = 11.01;      // inches between the center of the two wheels
double r = 6.0;                  // inches for circle radius

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
//float KpForward = 0.10136179, KiForward = 0.156809;
float KpForward = 0.06136179, KiForward = 0.1568089;
float IForward = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

// Rotational velocity PI controller variables ---------------------------------
float desiredRotational = 0.0, deltaRotational = 0.0;
float delta_v = 0.0;  //PI Output
//float KpRotational = 0.08, KiRotational = 0.8;
float KpRotational = 0.04369139, KiRotational = 0.8084569;
float IRotational = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

// Angle and Position controller variables ----------------------------------
float deltaPosition = 0.0, currentPosition = 0.0;
float deltaAngle = 0.0, currentAngle = 0.0;
float KpPos = 4, KpAng = 1.4;

// Motor Voltage Calcs-----------------------------------------------------
double v_a1 = (v_bar - delta_v) / 2;
double v_a2 = (v_bar + delta_v) / 2;

unsigned long timeBefore = micros();
unsigned long timeNow = micros();

// Encoder setup ---------------------------------------------------------
Encoder myEnc1(1, 2); // wheel 1 (left)
Encoder myEnc2(3, 4); // wheel 2 (right)

// Misc Booleans ------------------------------------------------------------
byte data[32];
bool done = false;

#define SLAVE_ADDRESS 0
int i = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  delay(100);

  pinMode(driveModePin, OUTPUT);
  digitalWrite(driveModePin, HIGH);

  pinMode(m1DirPin, OUTPUT);
  pinMode(m1SpeedPin, OUTPUT);

  pinMode(m2DirPin, OUTPUT);
  pinMode(m2SpeedPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    receiveData();
  }
}

void receiveData() {
  data[0] = Serial.read();
  if (data[0] > 2) {
    for (i = 1; i < 6; i++) {
      while (Serial.available() == 0);
      data[i] = Serial.read();
    }
  }

  if (data[0] == 1) {
    moveRobot(0, 3.14159*2, 18, 0.4);
    if (done == false)
      moveRobot(0, 3.14159*2, 18, 0.3);
    done = true;
  }
  
  else if (data[0] == 3) {
    float angleCmd = (float)data[3] + (float)(data[4]) / 256;
    angleCmd = 3.14159 * angleCmd / 180;
    float distanceCmd = (float)data[1] + (float)(data[2]) / 256;
    moveRobot(0, angleCmd, 20, 5);
    moveRobot(distanceCmd, 0, 20, 5);
    Serial.write(1);
  }
  
  else if (data[0] == 4) {
    float angleCmd = 0.0 - ((float)data[3] + (float)(data[4]) / 256);
    angleCmd = 3.14159 * angleCmd / 180;
    Serial.println(angleCmd);
    float distanceCmd = (float)data[1] + (float)(data[2]) / 256;
    Serial.println(distanceCmd);
    moveRobot(0, angleCmd, 20, 5);
    moveRobot(distanceCmd, 0, 20, 5);
    Serial.write(1);
  }

  else if (data[0] == 2) {
    driveCircle();
    if (done == false)
      driveCircle();
    done = true;
  }
  
  
  analogWrite(m1SpeedPin, 0);
  analogWrite(m2SpeedPin, 0);
  Serial.flush();
}
