#include <Wire.h>
#include <Encoder.h>
#define SLAVE_ADDRESS 0x04

/* Name: Group 6 | EENG350 | Mini Project: 4.6

   Purpose: Step response experiment

   How to Use:

   NOTE: This hasn't been tested at all and probably doesn't even work right now. 
   I just ran out of time and can't work on it anymore tonight.

*/

int sampleRate = 1; // 5 milliseconds
int desiredPWM = 255;

int driveModePin = 4;    // drive/break mode when HIGH, drive/coast mode when LOW

int m1DirPin = 7;       // sign of voltage
int m1SpeedPin = 9;     // voltage

byte data[32];
byte posArray[3] = {2, 0, 0};

int pos = 0;
int lastPos = 0;
double angularVelocity = 0.0;
int i = 0;
int j = 0;

Encoder myEnc(2,3);

void setup() {
  Serial.begin(115200); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  pinMode(driveModePin, OUTPUT);
  digitalWrite(driveModePin, HIGH);

  pinMode(m1DirPin, OUTPUT);
  digitalWrite(m1DirPin, HIGH);

  pinMode(m1SpeedPin, OUTPUT);
}

void loop() {
  unsigned long timeBefore = micros();

  make_step();
  // Read encoder and stuff
  lastPos = pos;
  pos = abs(myEnc.read() % 3200);
  posArray[1] = pos & 255;
  posArray[2] = pos / 256;
  angularVelocity = ((double) (pos - lastPos)) * 2.0 * 3.14159 / 3200.0 / (0.001 * sampleRate);
  
  unsigned long timeNow = micros();
  unsigned long timeMain = timeNow - timeBefore; // total time spent doing things

  if ( timeMain > (sampleRate * pow(10, 3)) ) {
    Serial.println("Main takes too long.");
  } else {
    while ( micros() < timeNow + (sampleRate * pow(10, 3) - timeMain) ); // take a sample every 5ms = 5000us
    
    // Insert serial data here (time, motor voltage, angular velocity)
    //Serial.print( millis() );
    //Serial.print(" ");
    //Serial.print(analogRead(m1SpeedPin));
    Serial.print(" ");
    Serial.println(angularVelocity);
    
  }
}

void make_step() { // Sets motor speed to 0 before 1 sec, then a positive speed after 1 sec
  if (millis() < 1000) {
    analogWrite(m1SpeedPin, 0);
  } else {
    analogWrite(m1SpeedPin, desiredPWM);
  }
}
 
// callback for received data
void receiveData(int byteCount){
  i = 0;
  while(Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  i--;
  
}

void sendData() {
  if (i == 0) {
    if (data[0] == 1) {
      j = 0;
      data[0] = 0;
    }
    else {
      j++;
    }
    Wire.write(posArray[j]);
  }
}
