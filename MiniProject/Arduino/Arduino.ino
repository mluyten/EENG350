#include <Wire.h>
#include <Encoder.h>
#define SLAVE_ADDRESS 0x04

/* Name: Group 6 | EENG350 | Mini Project: 4.6

  Purpose: Step response experiment

  How to Use:

  NOTE: This hasn't been tested at all and probably doesn't even work right now.
  I just ran out of time and can't work on it anymore tonight.

*/

int sampleRate = 5; // milliseconds
int desiredPWM = 255;

int driveModePin = 4;    // drive/break mode when HIGH, drive/coast mode when LOW

int m1DirPin = 7;       // sign of voltage
int m1SpeedPin = 9;     // voltage

byte data[32];
byte posArray[3] = {
  2, 0, 0
};

int thetaCurrent = 0;
int thetaSet = 2700;
double angularVelocity = 0.0;
int cmd = 0;
boolean cmdReceived = false;
int i = 0;
int j = 0;


float u = 0.0;  //PI Output
float Kp = 0.1071, Kd = 0, Ki = 0.00291;                    //Controller parameters
//float Kp = 0.25701, Kd = 0, Ki = 0.00291;
float I = 0.0, thetaDelta = 0.0;    //Variables that the program calculated as coefficients for the controller parameters
unsigned long Ts = 0, Tc = millis();

Encoder myEnc(2, 3);

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

  // Read encoder and stuff
  thetaCurrent = myEnc.read();
  
  if (thetaCurrent < 0) {
   thetaCurrent = abs(thetaCurrent + 3200);
  }
  else {
    thetaCurrent = thetaCurrent % 3200;
  }
  
  posArray[1] = thetaCurrent & 255;
  posArray[2] = thetaCurrent / 256;
  //angularVelocity = ((double) (thetaCurrent - lastThetaCurremt)) * 2.0 * 3.14159 / 3200.0 / (0.001 * sampleRate);

  thetaDelta = thetaSet - thetaCurrent;
  
  if (thetaDelta > 1600) {
    thetaDelta = thetaDelta - 3200;
  }
  else if (thetaDelta < -1600) {
    thetaDelta = thetaDelta + 3200;
  }

  if ((thetaDelta > 5) || (thetaDelta < -5)) {

    I = I + sampleRate * thetaDelta;                          // Define the integral term
  
    u = Kp * thetaDelta + Ki * 2;            // Put it all together to get the resulting position change
    analogWrite(m1SpeedPin, u + 15);   // Output this result to the motor
  
    Serial.print(u);
    Serial.print("\t");
    Serial.println(thetaDelta);
    
    if (u < 0) {
      digitalWrite(m1DirPin, HIGH);
    } else {
      digitalWrite(m1DirPin, LOW);
    }
  }
  else {
    analogWrite(m1SpeedPin, 0);
  }

  unsigned long timeNow = micros();
  unsigned long timeMain = timeNow - timeBefore; // total time spent doing things

  if ( timeMain > (sampleRate * pow(10, 3)) ) {
    Serial.println("Main takes too long.");
  }
  else {
    while ( micros() < timeNow + (sampleRate * pow(10, 3) - timeMain) ); // take a sample every 5ms = 5000us

  }
}

void make_step() { // Sets motor speed to 0 before 1 sec, then a positive speed after 1 sec
  if (millis() < 1000) {
    analogWrite(m1SpeedPin, 0);
  }
  else {
    analogWrite(m1SpeedPin, desiredPWM);
  }
}

// callback for received data
void receiveData(int byteCount) {
  i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  i--;

  if ((data[1] != cmd) && (data[1] != 0)) {
    cmdReceived = true;
    cmd = data[1];
  }

}

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
