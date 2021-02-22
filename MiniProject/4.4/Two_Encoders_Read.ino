/* Name: Linlee Morrison | EENG350 | Assignment 2: Two Encoders Read

   Purpose: To print either the position of the two "wheels", the current velocity of each of the "wheels", or the x distance, y distance, and angle of the robot.
            Using the velocity functionality, the output to the serial monitor can be copied into the accompanying "animation_robot2.m" file to create an animation of the robot.

   How to Use: Hook up both rotary encoders + and GND to power and ground, and then SW --> nothing, DT1 --> 2 and CLK1 --> 4, DT2 --> 3 and CLK2 --> 5

*/

const int A1_Pin = 2;                   //channel A of encoder 1 is plugged into digital pin 2, for interrupts
const int A2_Pin = 3;                   //channel A of encoder 2 is plugged into digital pin 3, for interrupts
const int B1_Pin = 4;                   //channel B of encoder 1 is plugged into digital 4
const int B2_Pin = 5;                   //channel B of encoder 2 is plugged into digital 5

int channelA1 = 0;                      // global variables for default values of channels A and B for both encoders
int channelB1 = 0;
int channelA2 = 0;
int channelB2 = 0;

const float pi = 3.14;
const float N = 40.0;                   // total counts for one complete rotation
const float r = 0.05;                   // wheel radius
const float b = 0.1;                    // wheel baseline (distance between wheels)

volatile double position1 = 0.0;        // global variables to keep track of current and past positions of both encoders
volatile double oldPosition1 = 0.0;
volatile double position2 = 0.0;
volatile double oldPosition2 = 0.0;

double phi_old = 0.0;                   // default angle, position variables
double x_old = 0.0;
double y_old = 0.0;

volatile double velocity1 = 0.0;        // velocities of both wheels, set in interrupts
volatile double velocity2 = 0.0;

unsigned long previousTime = 0;         // for loop timer in main
unsigned long timer1Flag = 0;           // checked in main loop to see when encoder1CompareISR was called last
unsigned long timer2Flag = 0;           // checked in main loop to see when encoder2CompareISR was called last


// ISR for Encoder 1 ------------------------------
void encoder1CompareISR() {             // tied to A1_Pin, called when A changes
  static unsigned long lastCall = 0;    // creates a variable to keep track of the last time the ISR was called (via A_pin changing)
  unsigned long newCall = micros();     // assigns the current run time of the program to a variable marking the timestamp of the current call, in microseconds
  static int counter1 = 0;              // New counter just for rotations of encoder 1
  int direction1 = 0;                   // Used for determining positive/negative velocity

  timer1Flag = newCall * pow(10, -3);   // Keeps track of the last time (in milliseconds) the ISR was called to be used in main

  if (newCall - lastCall > 50000) {     // see if the difference between the calls is more than 50ms to eliminate bouncing
    channelA1 = digitalRead(A1_Pin);    // Read values from the two channels
    channelB1 = digitalRead(B1_Pin);

    if (channelA1 == channelB1) {       //given that A is changing, clockwise 2 counts, set direction to positive
      counter1 += 2;
      direction1 = 1;
    } else {                            // A and B are different, counterclockwise 2 counts, set direction to negative
      counter1 -= 2;
      direction1 = -1;
    }

    position1 = (counter1 * 2.0 * pi ) / N;                                             // calculate position in radians
    velocity1 = (4.0 * pi * direction1) / ((newCall - lastCall) * pow(10, -6) * N);     // calculate velocity using difference between interrupt call times
  }
  lastCall = newCall;                  // sets time of the last ISR call to the current call time
}

// ISR for Encoder 2 ------------------------------
void encoder2CompareISR() {             // tied to A2_Pin, called when A changes, everything else is the same as the other ISR except applied to encoder 2
  static unsigned long lastCall = 0;
  unsigned long newCall = micros();
  static int counter2 = 0;
  int direction2 = 0;

  timer2Flag = newCall * pow(10, -3);

  if (newCall - lastCall > 70000) {     // currently a slightly longer debouncing time because this encoder is worse
    channelA2 = digitalRead(A2_Pin);
    channelB2 = digitalRead(B2_Pin);

    if (channelA2 == channelB2) {
      counter2 += 2;
      direction2 = 1;
    } else {
      counter2 -= 2;
      direction2 = -1;
    }

    position2 = (counter2 * 2.0 * pi ) / N;
    velocity2 = (4.0 * pi * direction2) / ((newCall - lastCall) * pow(10, -6) * N);
  }
  lastCall = newCall;

}

// Setup -------------------------------------------
void setup() {
  Serial.begin(9600);                   // set up serial communications, baud 9600

  pinMode(A1_Pin, INPUT_PULLUP);        // Configures A and B to have pullup resistors (HIGH when disconnected, LOW when connected)
  pinMode(B1_Pin, INPUT_PULLUP);
  pinMode(A2_Pin, INPUT_PULLUP);
  pinMode(B2_Pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(A1_Pin), encoder1CompareISR, CHANGE); // interrupt triggers when channel A1 changes
  attachInterrupt(digitalPinToInterrupt(A2_Pin), encoder2CompareISR, CHANGE); // interrupt triggers when channel A2 changes
}

// Main Loop ---------------------------------------
void loop() {
  unsigned long timeNow = millis();              // We're using the millis() function this time around to get away from the blocking delay() function

  // Uncomment me to see x, y, and phi values
  /*if ( (timeNow - previousTime) > 1000 ) {     // Values are checked every 1s
    previousTime = timeNow;

    double deltaP1 = position1 - oldPosition1;   // Calculate the change in positions of both "wheels"
    double deltaP2 = position2 - oldPosition2;

    Serial.print( xDistance(deltaP1, deltaP2) ); // Print out the x, y, and phi values of the robot, using values from the x, y, and phi helper functions
    Serial.print("        ");
    Serial.print( yDistance(deltaP1, deltaP2) );
    Serial.print("        ");
    Serial.println( anglePhi(deltaP1, deltaP2) );

    oldPosition1 = position1;                     // Set the previous position to the current position to get new deltaPs
    oldPosition2 = position2;
    }*/

  // Uncomment me to see time and left and right velocity values (for MATLAB animation)
  if ( (timeNow - previousTime) > 100 ) {         // Values are checked every 100ms
    previousTime = timeNow;
    double t = timeNow * pow(10, -3);             // Create a time variable that will display the time this function was called in seconds

    if (timeNow - timer1Flag > 250) {             // Check when the last time encoder1CompareISR was called
      velocity1 = 0;                              // Set the velocity of that encoder to 0 if it has been more than 250ms since the ISR was called (encoder hasn't moved in +250ms)
    }
    if (timeNow - timer2Flag > 250) {             // Do the same thing for encoder2CompareISR
      velocity2 = 0;
    }

    Serial.print(t);                              //prints time in seconds and velocities of the left and right wheels, separated by tabs
    Serial.print("\t");
    Serial.print(velocity1);
    Serial.print("\t");
    Serial.println(velocity2);

  }
  
}

// Helper functions for x, y, and phi calculations --------------------------------
double xDistance(double delta_dl, double delta_dr) {
  double x_new = x_old + cos(phi_old) * (delta_dl + delta_dr) / 2.0; // Take an Euler approximation
  x_old = x_new;                                                     // update variable
  return x_new;                                                      // return value to be printed
}

double yDistance(double delta_dl, double delta_dr) {
  double y_new = y_old + sin(phi_old) * (delta_dl + delta_dr) / 2.0;
  y_old = y_new;
  return y_new;
}

double anglePhi(double delta_dl, double delta_dr) {
  double phi_new = phi_old + (r / b) * (delta_dl - delta_dr); 
  phi_old = phi_new; 
  return phi_new;
}
