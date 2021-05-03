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
