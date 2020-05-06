%   FloatShield closed-loop LQ response example
%
%   LQ feedback control of ball altitude in the FloatShield.
%
%   This example initialises and calibrates the board then lifts
%   the ball off the ground and starts LQ control using predefined
%   reference trajectory. At the end of trajectory the results
%   are stored in 'responseLQ.mat' file and are shown on the figure
%   on the screen.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Peter Chmurciak.
%   Last update: 24.4.2020.

clc; clear; close all;              % Clears command window, variables and opened figures
clear estimateKalmanState;          % Clears persistent variables in estimate function

FloatShield = FloatShield;          % Create FloatShield object from FloatShield class
FloatShield.begin('COM4', 'UNO');   % Initialise shield with used Port and Board type
FloatShield.calibrate();            % Calibrate FloatShield

Ts = 0.025;             % Sampling period in seconds
k = 1;                  % Algorithm step counter
nextStep = 0;           % Algorithm step flag
samplingViolation = 0;  % Sampling violation flag

Ref = [210,160,110,145,195,245,180,130,65,95]; % Reference trajectory in mm
T = 1200;                           % Section length
i = 0;                              % Section counter
response = zeros(length(Ref)*T, 3); % Preallocate output variable

X = [0; 0; 0; 0];       % State vector 
Xr = [0; 0; 0; 0];      % Reference vector 

% Load system state-space matrices from FloatShield_LQ_Gain example
load FloatShield_LinearSS_Discrete_Matrices_25ms

% Create modified system state-space matrices that include integrator state
matAhat = [matA, zeros(3, 1); -matC, 1];
matBhat = [matB; 0];

% Penalisation matrices
Q = diag([1, 1, 1e7, 1e2]);     % State penalisation 
R = 1e7;                        % Input penalisation

% Calculate LQ gain that includes integrator state
K = dlqr(matAhat, matBhat, Q, R);

while (1)               % Lift the ball off the ground
    Xr(1) = Ref(1);     % Set reference to the first point in trajectory
    y = FloatShield.sensorReadAltitude();   % Read ball position
    u = -K * (X - Xr);                      % Calculate LQ system input
    FloatShield.actuatorWrite(u);           % Actuate
    % Estimate first three states with Kalman filter
    X(1:3) = estimateKalmanState(u, y, matA, matB, matC, Q_Kalman, R_Kalman); 
    X(4) = X(4) + (Xr(1) - X(1));   % Increment fourth (sum) state
    if (y >= Xr(1) * 2 / 3)         % If the ball is getting close to reference
        break                       % Continue program
    end
    pause(Ts)                       % Wait before repeating the loop
end

tic                                 % Start measuring time
while (1)                           % Infinite loop
    if (nextStep)                   % If step flag is enabled
        if (mod(k, T*i) == 1)       % At the end of section, progress in trajectory
            i = i + 1;
            if (i > length(Ref))    % If at the end of trajectory
                FloatShield.actuatorWrite(0.0);
                break               % Stop the program execution
            end
            Xr(1) = Ref(i);         % Progress in trajectory
        end
        y = FloatShield.sensorReadAltitude();   % Read ball position
        u = -K * (X - Xr);                      % Calculate LQ system input
        FloatShield.actuatorWrite(u);           % Actuate
        % Estimate first three states with Kalman filter
        X(1:3) = estimateKalmanState(u, y, matA, matB, matC, Q_Kalman, R_Kalman);
        X(4) = X(4) + (Xr(1) - X(1));           % Increment fourth (sum) state
        response(k, :) = [Xr(1), y, u];         % Store results
        k = k + 1;                              % Increment step counter
        nextStep = 0;                           % Disable step flag
    end                                         % Step end
    
    if (toc >= Ts * k)              % If its time for next sample
        if (toc >= Ts * (k + 1))    % Check if steps overlap
            disp('Sampling violation has occured.')
            samplingViolation = 1   % Set sampling violation flag            
            FloatShield.actuatorWrite(0);
            break                   % Stop program if they do overlap
        end
        nextStep = 1;               % Enable step flag
    end
end
response = response(1:k-1, :);      % Remove unused space
save responseLQ response            % Save results in responseLQ.mat file
disp('The example finished its trajectory. Results have been saved to "responseLQ.mat" file.')
plotResults('responseLQ.mat')       % Plot results from responseLQ.mat file