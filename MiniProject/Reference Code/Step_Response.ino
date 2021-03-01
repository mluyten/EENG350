/* Name: Group 6 | EENG350 | Mini Project: 4.6

   Purpose: Step response experiment (old).
   Working step response code can be found in MP_Final_Arduino.ino

*/

int sampleRate = 5; // 5 milliseconds
int desiredPWM = 128;

int driveModePin = 4;    // drive/break mode when HIGH, drive/coast mode when LOW

int m1DirPin = 7;       // sign of voltage
int m1SpeedPin = 9;     // voltage

void setup() {
  Serial.begin(9600);

}

void loop() {
  unsigned long timeBefore = micros();

  make_step();
  // Read encoder and stuff

  unsigned long timeNow = micros();
  unsigned long timeMain = timeNow - timeBefore; // total time spent doing things

  if ( timeMain > (sampleRate * pow(10, 3)) ) {
    Serial.println("Main takes too long.");
  } else if ( micros() >= timeNow + (sampleRate * pow(10, 3) - timeMain) ) {
    // take a sample every 5ms = 5000us
  }
}


void make_step() { // Sets motor speed to 0 before 1 sec, then a positive speed after 1 sec
  if (millis() < 1000) {
    analogWrite(m1SpeedPin, 0);
  } else {
    analogWrite(m1SpeedPin, desiredPWM);
  }
}
