%   PressureShield closed-loop LQ response example
%
%   LQ feedback control of overpressure in the PressureShield.
%
%   This example initialises and calibrates the board 
%   and starts linear quadratic (LQ) control 
%   using a pre-defined reference. At the end 
%   the results are stored in 'responseLQ.mat' file and are shown on the 
%   figure on the screen.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Martin Staro?.
%   Last update: 16.5.2021.

startScript;                % Clears command window, variables and opened figures
clear estimateKalmanState;  % Clears persistent variables in estimate function

PressureShield = PressureShield;   % Create PressureShield object from PressureShield class
PressureShield.begin;              % Initialises shield

Ts = 0.2;                           % Sampling period in seconds
k = 1;                              % Algorithm step counter
nextStep = 0;                       % Algorithm step flag
samplingViolation = 0;              % Sampling violation flag
init = PressureShield.sensorRead();

Ref = [50 70 40 60 80 60 30 70 60 50]; % Reference overpressure in hPa
T = 160;                            % Section length
i = 0;                              % Section counter
response = zeros(length(Ref)*T, 3); % Preallocate output variable

umin = 0;
umax = 100;

X = [0; 0; 0];                   % State vector 
Xr = [0; 0; 0];                  % Reference vector 

% System state-space matrices

matA = [0.993, 0.01526; -0.003223, 0.4206];
matB = [0.008751; -0.1501];
matC = [1.74, 0.8755];

% Penalisation matrices
Q_kalman = [0.05, 0; 0, 0.59];       % State penalisation 
R_kalman = 1;                        % Input penalisation

% Calculate LQ gain that includes integrator state
K = [1.2501, 0.05, -0.01];
init = PressureShield.sensorRead();

tic                                 % Start measuring time
while (1)                           % Infinite loop
    if (nextStep)                   % If step flag is enabled
        if (mod(k, T*i) == 1)       % At the end of section, progress in reference
            i = i + 1;
            if (i > length(Ref))    % If at the end of reference
                PressureShield.actuatorWrite(0.0);
                break               % Stop the program execution
            end
            Xr(1) = Ref(i);         % Progress in reference
        end
        y = double((PressureShield.sensorRead()-init)/100);   % Read overpressure
        u = -K * (X - Xr);                      % Calculate LQ system input
        u = constrain(u,umin,umax);
        PressureShield.actuatorWrite(u);           % Actuate

        X(1:2) = estimateKalmanState(u, y, matA, matB, matC, Q_kalman, R_kalman);
        X(3) = X(3) + (Xr(1) - X(1) - Xr(1)*0.36);     
        response(k, :) = [Xr(1), y, u];         % Store results
        k = k + 1;                              % Increment step counter
        nextStep = 0;                           % Disable step flag
    end                                         % Step end
    
            if (toc>=800)
             samplingViolation = 1   % Set sampling violation flag            
             PressureShield.actuatorWrite(0);
             break                   % Stop program if they do overlap
         end
        nextStep = 1;               % Enable step flag
end
response = response(1:k-1, :);      % Remove unused space
save responseLQ response           % Save results in responseLQ.mat file
plotResults('responseLQ.mat')       % Plot results from responseLQ.mat file