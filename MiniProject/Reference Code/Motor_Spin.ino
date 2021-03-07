/* Name: Group 6 | EENG350 | Mini Project: 4.3

   Purpose: Make the motor spin

   How to Use: Supply power to the motor shield. 
   Plug the red and black wires from the motor into the M1 ports on the motor shield.
   Upload code to Arduino, make sure switch is flipped to provide power.

*/

int driveModePin = 4; // drive/break mode when HIGH, drive/coast mode when LOW

int m1DirPin = 7;       // sign of voltage
int m1SpeedPin = 9;     // voltage

int m2DirPin = 8;        // sign of voltage
int m2SpeedPin = 10;     // voltage

int inputPin = 12;

// desiredPWM = pulseWidth*256
int desiredPWM = 128; // values go from 0 to 255 for analogWrite


void setup() {
  pinMode(driveModePin, OUTPUT);
  digitalWrite(driveModePin, HIGH);
  
  pinMode(m1DirPin, OUTPUT);
  pinMode(m1SpeedPin, OUTPUT);
  pinMode(m2DirPin, OUTPUT);
  pinMode(m2SpeedPin, OUTPUT);

  pinMode(inputPin, INPUT);
  

}

void loop() {
  digitalWrite(m1DirPin, HIGH); // go forwards (positive)
  analogWrite(m1SpeedPin, desiredPWM); // spin at desired speed

}
