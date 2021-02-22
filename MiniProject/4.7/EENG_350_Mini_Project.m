%% Mini Project 4.7
% Simulates the motor turning as a result of an input PWM signal that is
% approxamated as a average of the actual PWM in Simulink.


%%%% G(s) = V_a/theta. The resulting change in position for a given input voltage

% Bad variables that are used to represent the actual performance of the system
Ra = 1; % System resistance
Kt = 0.5; 
Ke = 0.5;
J = 0.0323; % Inertia of the wheel
b = 0.5; % Unknown bearing friction

% Variables that we try to match to fit the performance of the 'bad
% varibles' to the simpler equation: K*sigma/(s+sigma) * 1/s
sigma = 1.1;
K = 0.222;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

out = sim('Mini_Poject_4_7'); % run the simulation

clf; figure(1);
subplot(3,1,1);
plot(out.desiredPosition); title('Desired Position'); ylabel('Distance (m)') 

subplot(3,1,2);
plot(out.actualPosition); title('Actual Position'); ylabel('Distance (m)') 

subplot(3,1,3);
plot(out.Va); title('V_a'); ylabel('Controller Voltage (V)') 
