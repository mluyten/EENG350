%% Demo 1: Matlab Documentation
%
% EENG 350 February 26, 2021
%
% Luis Cisneros

%% Mini Project 4.6: Simulate and Tune the Model to Match the 'actual' Data
% The controller was designed using MATLAB to guess and check sigma and K values 
% that could be used to simplify the transfer function for the step response 
% recorded from the motor step response data. This is done graphically by 
% overlaying the two graphs to determine the sigma and K values that
% best match the recorded motor performance.
% NOTE: The format of the 'actual' matrix should be [0,dataColumn]

%P:	0.0769387064810876
%I:	0.00198757916685295

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% User variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sampleRate = 1; % Number of MICROSECONDS between samples

sigma = 5; % Tune to match 'actual' plot
K = 20; % Gain. AKA the final velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add the time column to the 'actual' matrix (uncomment as needed)
load('actual.mat'); % Load the measured velocity values
%actuall(:,1) = (0 : 1 / (sampleRate * 100000) : (size(actual,1) - 1) / 1000000); % Use the sampling rate to create the time column
out = sim('StepResponse'); % Run the simulation and make the results part of the out. object

% Plot the results
clf; figure(1); hold on; xlim([0 2]);
plot(out.step * K); % Plot the step input
plot(out.newv,'m-'); % Plot the modeled step response
plot(actual(:,1),actual(:,2), 'b-'); ylabel('Velocity (rad/s)'); xlabel('Time (s)'); % Plot the actual step response
title('Open Looop Velocity'); legend({'Desired Velocity','Modeled Velocity','Actual Velocity'},'Location','southeast'); hold off;

%% Mini Project 4.7: Design a Controller for Your Motor
% Simulink was then used to tune a PI controller to the desired
% specifications of a rise time of 1 second and a percent overshoot of less
% than or equal to 12% while having zero steady state error. The PI
% controller that was selected had a rise time of exactly 1 second while
% having a percent overshoot of 1.58% as seen in the screenshot below.
% 
% The actual and the modeled closed loop performance differed only
% slightly as it got closer to steady state. The actual closed loop
% performance recorded is slightly faster than the modeled. This is likely due to
% the battery being charged after the initial step response was recorded, 
% and therefore having more power and turning the motor faster than
% expected.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% User variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sigma = 5;
K = 20;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%load('closedLoopActual.mat'); % Load the measured closed loop position values
%closedLoopActual(:,1) = closedLoopActual(:,1) / 1000; % Change the units from ms to seconds
%closedLoopActual(:,1) = closedLoopActual(:,1) + 1;
%closedLoopActual(:,2) = closedLoopActual(:,2)/max(closedLoopActual(:,2)); % Scale the max position to 1
out = sim('StepResponse'); % run the simulation

% Plot the results
clf; figure(2); hold on;
plot(out.thetaSet); % Plot the desired position
plot(out.u, 'm-'); % Plot the actual position
plot(closedLoopActual(:,1),closedLoopActual(:,2), 'b-'); ylabel('Distance (m)'); xlabel('Time (s)'); % Plot the actual step response
title('Controller Closed Loop Results'); legend({'Desired Position','Modeled Position','Actual Position'},'Location','east'); hold off;

%% Simulink Block Diagram
%
% <<Block_Diagram.png>>
% 
%% Simulink Tuned PI Performance
%
% <<Tuned_PI_Performance.png>>
% 
