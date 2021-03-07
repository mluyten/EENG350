/* Name: Group 6 | EENG350 | Mini Project: 4.6

   Purpose: Step response experiment

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

  make_step(); // Step response

  unsigned long timeNow = micros();
  unsigned long timeMain = timeNow - timeBefore; // total time spent doing things in main

  if ( timeMain > (sampleRate * pow(10, 3)) ) { // Error checking to make sure main doesn't take longer than the sampling rate
    Serial.println("Main takes too long.");
  }
  else {
    while ( micros() < timeNow + (sampleRate * pow(10, 3) - timeMain) ); // wait to take a sample exactly every 5ms = 5000us
         //Print samples here
  }


void make_step() { // Sets motor speed to 0 before 1 sec, then a positive speed after 1 sec
  if (millis() < 1000) {
    analogWrite(m1SpeedPin, 0);
  } else {
    analogWrite(m1SpeedPin, desiredPWM);
  }
}
