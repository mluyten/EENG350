#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

/* Name: Group 6 | EENG350 | Demo 2 Arduino Code

   Purpose: Drive in a circle
   
   Note: when viewed from the front, left wheel is 1, right wheel is 2. driveModePin is 4 on the motor shield, 5 on the Teensy
*/

// Test Sampling Method --------------------------------------------
unsigned long lastChange1 = 0;
unsigned long lastChange2 = 0;
bool velocityChanged = false;

int sampleRate = 5;            // 5 us
double radius = 3.0;           // inches
double wheelbase = 10.71;      // inches between the center of the two wheels
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
//float KpForward = 0.011211, KiForward = 0.05605;
float KpForward = 0.02136179, KiForward = 0.1068089;
float IForward = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

// Rotational velocity PI controller variables ---------------------------------
float desiredRotational = 0.0, deltaRotational = 0.0;
float delta_v = 0.0;  //PI Output
//float KpRotational = 0.035, KiRotational = 0.17501;
float KpRotational = 0.06369139, KiRotational = 0.2584569;
float IRotational = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

// Angle and Position controller variables ----------------------------------
float deltaPosition = 0.0, currentPosition = 0.0;
float deltaAngle = 0.0, currentAngle = 0.0;
float KpPos = 0.75, KpAng = 1;

// Motor Voltage Calcs-----------------------------------------------------
double v_a1 = (v_bar - delta_v) / 2;
double v_a2 = (v_bar + delta_v) / 2;

unsigned long timeBefore = micros();
unsigned long timeNow = micros();

// Encoder setup ---------------------------------------------------------
Encoder myEnc1(1, 2); // wheel 1 (left)
Encoder myEnc2(3, 4); // wheel 2 (right)

// Misc Booleans ------------------------------------------------------------
bool stop = false;
bool done = true;
byte data[32];

void setup() {
  Serial.begin(115200);

// initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  pinMode(driveModePin, OUTPUT);
  digitalWrite(driveModePin, HIGH);

  pinMode(m1DirPin, OUTPUT);
  pinMode(m1SpeedPin, OUTPUT);

  pinMode(m2DirPin, OUTPUT);
  pinMode(m2SpeedPin, OUTPUT);
}

void loop() {
  delay(1000);
  hardCodeCircle(); //moveRobot( (2*3.14*r) , (currentPosition/r) ); // go in a 1ft circle 
  analogWrite(m1SpeedPin, (0));
  analogWrite(m2SpeedPin, (0));
  while(true);
}

void hardCodeCircle(){
  timeBefore = millis(); //caution: reused variable from PID loop
  int deltaTime = 6000; //number of milliseconds of turning
  
  digitalWrite(m1DirPin, HIGH);
  digitalWrite(m2DirPin, HIGH);
  
  while(timeNow){        // caution: reused variable from PID loop
    timeNow = millis();
    analogWrite(m1SpeedPin, 100);
    analogWrite(m2SpeedPin, 150);
  }

  //stop everything after the time is over
  digitalWrite(m1DirPin, LOW);
  digitalWrite(m2DirPin, LOW);
}

void receiveData(int byteCount) {
  i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  i--;

  if (data[1] == 1) {
    moveRobot(0, 3.14159*4, 10, 2);
  }
  else if (data[1] == 2) {
    stop = true;
    done = false;
    moveRobot(0, (float)(data[4] + data[5] / 256), 18, 5);
    done = true;
  }
  else if (data[1] == 3) {
    stop = true;
    done = false;
    moveRobot((float)(data[2] + data[3] / 256) - 12, 0, 18, 5);
    done = true;
  }
}

void sendData() {
  if (i == 0) {
    if (data[0] == 255) {
      if (done)
        Wire.write(1);
      else
        Wire.write(0);
    }
  }
}

void moveRobot(float desiredPosition, float desiredAngle, float maxForwardVelocity, float maxRotationalVelocity) {
  
  clearControlVariables();
  myEnc1.write(0);
  myEnc2.write(0);
  
  // PID loop
  while ((abs(desiredPosition - currentPosition) > 0.05) || (abs(desiredAngle - currentAngle) > 0.01)) {
    if (stop) {
      stop = false;
      return;
    }

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
  
    if (lastThetaCurrent2 != thetaCurrent2) { // Updates wheel 2 variables whenever wheel 2 encoder counts changes
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
      Serial.print(abs(desiredPosition - currentPosition));
      Serial.print("\t");
      Serial.print(abs(desiredAngle - currentAngle));
      Serial.print("\t");
      Serial.println(v_bar);
    }
  
    currentPosition = (thetaCurrent1 + thetaCurrent2) * 3.14 * radius / 3200; // current position of the robot in inches
    if (desiredPosition > currentPosition)
    deltaPosition = desiredPosition - currentPosition; // Position error ie. distance (in inches) from desired position
    desiredForward = KpPos * (deltaPosition); // Forward velocity P controller. Takes difference in position and multiplies it by a proportional controller to get desired velocity
    if (desiredForward > maxForwardVelocity) // Saturation for position P controller. Ensures that desired forward velocity cannot be greater than physically possible
      desiredForward = maxForwardVelocity;
  
    deltaAngle = desiredAngle - currentAngle; // Angle error ie. distance (in radians) from desired angle
    desiredRotational = KpAng * (deltaAngle); // Rotational velocity P controller. Takes difference in angle and multiplies it by a proportional controller to get desired velocity
    if (desiredRotational > maxRotationalVelocity) // Saturation for angle P controller. Ensures that desired rotational velocity cannot be greater than physically possible
      desiredRotational = maxRotationalVelocity;
  
    deltaForward = desiredForward - forwardVelocity; // Forward velocity error ie difference (in in/s) between desired and actual forward velocity
    if (deltaPosition < 0) {
      IForward = IForward + 0.000001 * sampleRate * deltaForward * -1; // Integral of the forward velocity PI controller (in in)
      v_bar = -1 * (KpForward * deltaForward * -1 + KiForward * IForward); // V_bar voltage from forward velocity PI controller
    } else {
      IForward = IForward + 0.000001 * sampleRate * deltaForward; // Integral of the forward velocity PI controller (in in)
      v_bar = KpForward * deltaForward + KiForward * IForward; // V_bar voltage from forward velocity PI controller
    }
    
    deltaRotational = desiredRotational - rotationalVelocity; // Rotational velocity error ie difference (in in/s) between desired and actual forward velocity
    if (deltaAngle < 0) {
      IRotational = IRotational + 0.000001 * sampleRate * deltaRotational * -1; // Integral of the rotational velocity PI controller (in in)
      delta_v = -1 * (KpRotational * deltaRotational * -1 + KiRotational * IRotational); // delta_v voltage from forward velocity PI controller
    } else {
      IRotational = IRotational + 0.000001 * sampleRate * deltaRotational; // Integral of the rotational velocity PI controller (in in)
      delta_v = KpRotational * deltaRotational + KiRotational * IRotational; // delta_v voltage from forward velocity PI controller
    
    }
    
    v_a1 = (v_bar + delta_v) / 2; 
    v_a2 = (v_bar - delta_v) / 2;
  
    analogWrite(m1SpeedPin, ( (int) (abs(v_a1) * 256) + 15));
    analogWrite(m2SpeedPin, ( (int) (abs(v_a2) * 256) + 15));
  
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
      Serial.println("Loop Takes Too Long!");
    } else {
      while ( micros() < sampleRate + timeBefore ); // wait to take a sample exactly every 5us
    }
  }
}

void clearControlVariables() {
  thetaCurrent1 = 0;
  deltaTheta1 = 0;
  lastThetaCurrent1 = 0;
  angularVelocity1 = 0.0;

  thetaCurrent2 = 0;
  deltaTheta2 = 0;
  lastThetaCurrent2 = 0;
  angularVelocity2 = 0.0;

  rotationalVelocity = 0.0;
  forwardVelocity = 0.0;

  desiredForward = 0.0;
  deltaForward = 0.0;
  v_bar = 0.0;  //PI Output
  IForward = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

  desiredRotational = 0.0;
  deltaRotational = 0.0;
  delta_v = 0.0;  //PI Output
  IRotational = 0.0;    //Variables that the program calculated as coefficients for the controller parameters

  deltaPosition = 0.0;
  currentPosition = 0.0;
  deltaAngle = 0.0;
  currentAngle = 0.0;

  v_a1 = 0.0;
  v_a2 = 0.0;
}
