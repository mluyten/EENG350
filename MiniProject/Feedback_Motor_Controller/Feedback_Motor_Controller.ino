/*
   Feeback_Motor_Controller
   Created by Luis Cisneros
   February 24 2021
   EENG 350 Mini Project

   Description here
*/

float u = 0.0;  //PI Output
float Kp = 0.0871, Kd = 0, Ki = 0.00291;                    //Controller parameters
float I = 0.0, nowError = 0.0, pastError = 0.0, D = 0.0;    //Variables that the program calculated as coefficients for the controller parameters
float setPosition = 0.0, currentPosition = 0.0;
unsigned long Ts = 0, Tc = millis();


void setup() {
  Serial.begin(9600);
}


void loop() {
  Serial.print(setPosition);
  Serial.print(" ");
  Serial.print(currentPosition);
  Serial.print(" ");
  Serial.print(u / 10);
  Serial.println("");

  if (millis() > 5000)                                  // Set the setPosition variable after 1 second
    setPosition = 5.0;                                  //[TODO get desired position here]
  currentPosition = 5.0 * sin(millis() / 1000.0) + 5.0; //[TODO read current position here]

  nowError = setPosition - currentPosition;
  I = I + Ts * nowError;                          // Define the integral term
  if (Ts > 0) {                                   // Wait until the program has looped at least once
    D = (nowError - pastError) / Ts;              // Find the change in error to get the derivative term
    pastError = nowError;
  }
  
  u = Kp * nowError + Ki * I + Kd * D;            // Put it all together to get the resulting position change
  //analogWrite(2,u);                             // Output this result to the motor


  Ts = millis() - Tc;
  Tc = millis();
}
