#include <Wire.h>
#include <Encoder.h>
#define SLAVE_ADDRESS 0x04

/* Group 6 | EENG350 | Mini Project Arduino Code

  Purpose: Receives instructions from the Pi about what angle to rotate to.
  Implements the PI loop and rotates the motor accordingly. 
  Reads data from the motor encoder every 5ms.

  How to Use: Use the Mini Project motor setup. Start the Pi first, then upload the Arduino code.

*/

int sampleRate = 5;      // milliseconds

// Motor Values------------------------------------------------------------------
int desiredPWM = 255;

int driveModePin = 4;    // drive/break mode when HIGH, drive/coast mode when LOW

int m1DirPin = 7;       // sign of voltage
int m1SpeedPin = 9;     // voltage

// Pi Communication--------------------------------------------------------------
byte data[32];
byte posArray[3] = {
  2, 0, 0
};

// Angular Position and Velocity-------------------------------------------------
int thetaCurrent = 0;
int thetaSet = 0;
double angularVelocity = 0.0;
int i = 0;
int j = 0;
boolean encReset = true;

// PID Loop Variables------------------------------------------------------------
float u = 0.0;  //PI Output
//float Kp = 0.1071, Kd = 0, Ki = 0.00291;                    //Controller parameters
float Kp = 0.12638, Kd = 0, Ki = 0.0061089;
float I = 0.0, thetaDelta = 0.0;    //Variables that the program calculated as coefficients for the controller parameters
unsigned long Ts = 0, Tc = millis();

Encoder myEnc(2, 3);

//Setup Function********************************************************************
void setup() {
  Serial.begin(115200); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  // Motor setup: drive/break mode, default rotation is forwards
  pinMode(driveModePin, OUTPUT);
  digitalWrite(driveModePin, HIGH);

  pinMode(m1DirPin, OUTPUT);
  digitalWrite(m1DirPin, HIGH);

  pinMode(m1SpeedPin, OUTPUT);
} //End of Setup Function************************************************************

//Main Function**********************************************************************
void loop() {
  unsigned long timeBefore = micros();  // Get time before the loop starts

  // Encoder processing -------------------------------------------------------------
  thetaCurrent = myEnc.read();
  
  // Modify encoder value to be between 0 and 3200 counts (1 full rotation)
  if (thetaCurrent < 0) {
   thetaCurrent = abs(thetaCurrent + 3200) % 3200;
  }
  else {
    thetaCurrent = thetaCurrent % 3200;
  }
  
  posArray[1] = thetaCurrent & 255;
  posArray[2] = thetaCurrent / 256;
  //angularVelocity = ((double) (thetaCurrent - lastThetaCurremt)) * 2.0 * 3.14159 / 3200.0 / (0.001 * sampleRate);

  // change between desired number of counts (angle) and current number of counts (angle)
  thetaDelta = thetaSet - thetaCurrent;
  
  // Rotation handling: if thetaDelta is more than 1600 counts off, set it back into the -3200 to +3200 range
  if (thetaDelta > 1600) {
    thetaDelta = thetaDelta - 3200;
  }
  else if (thetaDelta < -1600) {
    thetaDelta = thetaDelta + 3200;
  }

  // PID Loop-------------------------------------------------------------------
  if ((thetaDelta > 5) || (thetaDelta < -5)) { // if the motor is not within a reasonable range of the desired value, pick a travel direction
    if (thetaDelta < 0) {
      digitalWrite(m1DirPin, HIGH);
    } else {
      digitalWrite(m1DirPin, LOW);
    }
    
    encReset = false;
    
    I = I + sampleRate * abs(thetaDelta);                          // Define the integral term
  
    u = Kp * abs(thetaDelta) + Ki * 2;            // Transfer Function: put it all together to get the resulting position change (as a PWM voltage value)
    analogWrite(m1SpeedPin, u + 15);              // Write the PWM value to the motor, with a little extra to compensate 
                                                  //  for small changes where the set voltage can't spin the motor
    
    //Debugging code to compare voltage values from the PID loop to the distance away from the desired angle
    Serial.print(u);                              
    Serial.print("\t");
    Serial.println(thetaDelta);
    
  }
  else { // motor has reached desired position, set encoder to current position and stop the motor
    if (!encReset) {
      myEnc.write(thetaCurrent);
    }
    analogWrite(m1SpeedPin, 0);
  }

  // Timing and Sampling----------------------------------------------------
  unsigned long timeNow = micros();
  unsigned long timeMain = timeNow - timeBefore; // total time spent doing things in main (encoder math, PID loop)

  if ( timeMain > (sampleRate * pow(10, 3)) ) { // Error checking to make sure main doesn't take longer than the sampling rate
    Serial.println("Main takes too long.");
  }
  else {
    while ( micros() < timeNow + (sampleRate * pow(10, 3) - timeMain) ); // wait to take a sample exactly every 5ms = 5000us

  }
}//End of Main Function******************************************************************************

// Step Function for development of PID loop--------------------------------------------------------
void make_step() { 
  if (millis() < 1000) {
    analogWrite(m1SpeedPin, 0);
  }
  else {
    analogWrite(m1SpeedPin, desiredPWM);
  }
}

// callback for received data----------------------------------------------------------------------
void receiveData(int byteCount) {
  i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  i--;

  if (data[1] == 1) {
    thetaSet = 800;
  }
  else if (data[1] == 2) {
    thetaSet = 1600;
  }
  else if (data[1] == 3) {
    thetaSet = 2400;
  }
  else if (data[1] == 4) {
    thetaSet = 0;
  }

}

// Send data to Pi----------------------------------------------------------------------------
void sendData() {
  if (i == 0) {
    if (data[0] == 255) {
      j = 0;
      data[0] = 0;
    }
    else {
      j++;
    }
    Wire.write(posArray[j]);
  }
}
