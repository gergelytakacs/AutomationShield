%   FloatShield closed-loop PID response example
%
%   PID feedback control of ball altitude in the FloatShield.
%
%   This example initialises and calibrates the board then lifts
%   the ball off the ground and starts PID control using predefined
%   reference trajectory. At the end of trajectory the results
%   are stored in 'responsePID.mat' file and are shown on the figure
%   on the screen.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Peter Chmurciak.
%   Last update: 23.4.2020.

clc; clear; close all;            % Clears command window, variables and opened figures

FloatShield = FloatShield;        % Create FloatShield object from FloatShield class
FloatShield.begin('COM4', 'UNO'); % Initialise shield with used Port and Board type
FloatShield.calibrate();          % Calibrate FloatShield
PID = PID;                        % Create PID object from PID class

Ts = 0.025;                       % Sampling period in seconds
k = 1;                            % Algorithm step counter
nextStep = 0;                     % Algorithm step flag
samplingViolation = 0;            % Sampling violation flag

R = [65, 50, 35, 45, 60, 75, 55, 40, 20, 30]; % Reference trajectory
T = 2400;                                     % Section length
i = 0;                                        % Section counter
response = zeros(length(R)*T, 3);             % Preallocate output variable

Kp = 0.25;                          % PID Gain
Ti = 5.0;                           % PID Integral time constant
Td = 0.01;                          % PID Derivative time constant
PID.setParameters(Kp, Ti, Td, Ts);  % Feed the constants to PID object 

while (1)                                     % Lift the ball off the ground
    r = R(1);                                 % Set reference to the first point in trajectory
    y = FloatShield.sensorRead();             % Read ball position
    u = PID.compute(r-y, 30, 100, 30, 100);   % Compute PID response
    FloatShield.actuatorWrite(u);             % Actuate
    if (y >= r * 2/3)                         % If the ball is getting close to reference
        break                                 % Continue program
    end
    pause(Ts)                                 % Wait before repeating the loop
end

tic                                          % Start measuring time
while (1)                                    % Infinite loop
    if (nextStep)                            % If step flag is enabled
        if (mod(k, T*i) == 1)                % At the end of section, progress in trajectory
            i = i + 1;                       
            if (i > length(R))               % If at the end of trajectory
                FloatShield.actuatorWrite(0.0);
                break                        % Stop the program execution
            end
            r = R(i);                        % Progress in trajectory  
        end
        y = FloatShield.sensorRead();          % Read ball position
        u = PID.compute(r-y, 0, 100, 0, 100);  % Compute PID response
        FloatShield.actuatorWrite(u);          % Actuate
        response(k, :) = [r, y, u];            % Store results
        k = k + 1;                             % Increment step counter
        nextStep = 0;                          % Disable step flag
    end                                        % Step end
    
    if (toc >= Ts * k)                            % If its time for next sample
        if (toc >= Ts * (k + 1))                  % Check if steps overlap
            disp('Sampling violation has occured.')
            samplingViolation = 1                 % Set sampling violation flag            
            FloatShield.actuatorWrite(0);             
            break                                 % Stop program if they do overlap
        end
        nextStep = 1;                             % Enable step flag
    end
end
response = response(1:k-1, :);                    % Remove unused space
save responsePID response                         % Save results in responsePID.mat file
disp('The example finished its trajectory. Results have been saved to "responsePID.mat" file.')
plotPIDResponse('responsePID.mat')                % Plot results from responsePID.mat file