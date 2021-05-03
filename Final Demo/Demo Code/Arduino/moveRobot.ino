void moveRobot(float desiredPosition, float desiredAngle, float maxForward, float maxRotational) {
  
  clearControlVariables();
  int positionCounter = 0;
  bool inFinalPosition = false;
  myEnc1.write(0);
  myEnc2.write(0);

  while (positionCounter < 100) {
    // Robot Receives a Stop Command ------------------------------------------------------------------------------------------------------------
    if ((Serial.available() > 0) && (Serial.read() == 0)) {
      analogWrite(m1SpeedPin, 0);
      analogWrite(m2SpeedPin, 0);
      done = true;
      return;
    }
    
  // PID loop
    timeBefore = micros();
    
    thetaCurrent1 = -1 * myEnc1.read(); //Switch polarity of counts.
    thetaCurrent2 = myEnc2.read();
    
  // Calculate Current Velocity -------------------------------------------------------------------------------------------------------------------
    if (lastThetaCurrent1 != thetaCurrent1) { // Updates wheel 1 variables whenever wheel 1 encoder counts changes
      deltaTheta1 = (thetaCurrent1 - lastThetaCurrent1); // Change in encoder counts
      angularVelocity1 = (double) deltaTheta1 * 2.0 * 3.14159 / 3200.0 / (0.000001 * (micros() - lastChange1)); // Wheel 1 angular velocity in rad/s
      lastChange1 = micros(); // Updates time when the wheel 1 variables were updated
      lastThetaCurrent1 = thetaCurrent1; // Updates current encoder counts
      velocityChanged = true; // Enters velocity calculation if statement
    }
  
    if (lastThetaCurrent2 != thetaCurrent2) { // Updates wheel 2 variables whenever wheel 1 encoder counts changes
      deltaTheta2 = (thetaCurrent2 - lastThetaCurrent2); // Change in encoder counts
      angularVelocity2 = (double) deltaTheta2 * 2.0 * 3.14159 / 3200.0 / (0.000001 * (micros() - lastChange2)); // Wheel 2 angular velocity in rad/s
      lastChange2 = micros(); // Updates time when the wheel 1 variables were updated
      lastThetaCurrent2 = thetaCurrent2; // Updates current encoder counts
      velocityChanged = true; // Enters velocity calculation if statement
    }
  
    if (velocityChanged) { // If encoder counts changes on either wheel (if wheel moves), update velocity variables
      rotationalVelocity = ( (double) radius * (angularVelocity1 - angularVelocity2) / wheelbase ); // Robot's rotational velocity in rad/s
      forwardVelocity = ( (double) radius * (angularVelocity1 + angularVelocity2) / 2 ); // Robot's forward velocity in in/s
      velocityChanged = false; // ensure this loop does not execute unless encoder counts changes
      currentAngle = currentAngle + ( (double) radius * (deltaTheta1 - deltaTheta2) * 3.14159 / 3200.0 / wheelbase); // Updates the current angle of the robot in rad
      //Serial.print(abs(deltaPosition));
      //Serial.print("\t");
      //Serial.print(abs(deltaAngle));
      //Serial.print("\t");
      //Serial.println(v_bar);
    }
    
  // Outer Loop (Positional control)---------------------------------------------------------------------------------------------------------------------
    currentPosition = (thetaCurrent1 + thetaCurrent2) * 3.14 * radius / 3200; // current position of the robot in inches
    deltaPosition = desiredPosition - currentPosition; // Position error ie. distance (in inches) from desired position
    desiredForward = KpPos * (deltaPosition); // Forward velocity P controller. Takes difference in position and multiplies it by a proportional controller to get desired velocity
    if (desiredForward > maxForward) // Saturation for position P controller. Ensures that desired forward velocity cannot be greater than physically possible
      desiredForward = maxForward;
  
    deltaAngle = desiredAngle - currentAngle; // Angle error ie. distance (in radians) from desired angle
    desiredRotational = KpAng * (deltaAngle); // Rotational velocity P controller. Takes difference in angle and multiplies it by a proportional controller to get desired velocity
    if (desiredRotational > maxRotational) // Saturation for angle P controller. Ensures that desired rotational velocity cannot be greater than physically possible
      desiredRotational = maxRotational;

  // Inner Loop (Velocity Control)-----------------------------------------------------------------------------------------------------------------------
    deltaForward = desiredForward - forwardVelocity; // Forward velocity error ie difference (in in/s) between desired and actual forward velocity
    if (deltaPosition < 0) { //maybe take a second look at next line?
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

  // Convert to motor voltages------------------------------------------------------------------------------------------------------------------------
    v_a1 = (v_bar + delta_v) / 2; 
    v_a2 = (v_bar - delta_v) / 2;

  // Write speed to motors-------------------------------------------------------------------------------------------------------------------------
    analogWrite(m1SpeedPin, ( (int) (abs(v_a1) * 256) + 10));
    analogWrite(m2SpeedPin, ( (int) (abs(v_a2) * 256) + 10));

  // Write direction to motors-----------------------------------------------------------------------------------------------------------------------
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

  // Close Enough----------------------------------------------------------------------------------------------------------------------------------  
    if ((abs(deltaPosition) < 0.2) && (abs(deltaAngle) <= 0.05)) {
      if (inFinalPosition) {
        positionCounter++;
        inFinalPosition = true;
      } else {
        positionCounter = 1;
        inFinalPosition = true;
      }
    } else {
      positionCounter = 0;
    }

    timeNow = micros();
  
    if ( timeNow - timeBefore > sampleRate ) { // Error checking to make sure main doesn't take longer than the sampling rate
      Serial.println("Loop Takes Too Long!");
    } else {
      while ( micros() < sampleRate + timeBefore ); // wait to take a sample exactly every 5us
    }
  }
  analogWrite(m1SpeedPin, 0);
  analogWrite(m2SpeedPin, 0);
}
