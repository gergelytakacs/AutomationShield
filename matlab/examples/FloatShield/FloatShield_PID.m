%   FloatShield closed-loop PID response example
%
%   PID feedback control of ball altitude in the FloatShield.
%
%   This example initializes and calibrates the board then lifts
%   the ball off the ground and starts PID control using predefined
%   reference trajectory. At the end of trajectory the results
%   are stored in 'response.mat' file and are shown on the figure
%   on the screen.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Peter Chmurciak.
%   Last update: 10.7.2019.

clc; clear; close all;            % Clears command window, variables and opened figures

FloatShield = FloatShield;        % Create FloatShield object from FloatShield class
FloatShield.begin('COM3', 'UNO'); % Initialize shield with used Port and Board type
FloatShield.calibrate();          % Calibrate FloatShield
PID = PID;                        % Create PID object from PID class

Ts = 0.025;                       % Sampling period in seconds
k = 1;                            % Algorithm step counter
nextStep = 0;                     % Algorithm step flag
realTimeViolation = 0;            % Real-time violation flag

R = [70, 25, 5, 60, 30, 70, 50, 75, 80, 45];  % Reference trajectory
T = 1000;                                     % Section length
i = 0;                                        % Section counter

Kp = 0.25;                          % PID Gain
Ti = 5.0;                           % PID Integral time constant
Td = 0.01;                          % PID Derivative time constant
PID.setParameters(Kp, Ti, Td, Ts);  % Feed the constants to PID object 

while (1)                                     % Lift the ball off the ground
    r = R(1);                                 % Set reference to the first point in trajectory
    y = FloatShield.sensorRead();             % Read ball position
    u = PID.compute(r-y, 30, 100, 30, 100);   % Compute PID response
    FloatShield.actuatorWrite(u);             % Actuate
    if (y >= r / 2)                           % If the ball is getting close to reference
        break                                 % Continue program
    end
    pause(0.05)                               % Wait 50 miliseconds before repeating
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
            disp('Real-time sampling violation.')
            realTimeViolation = 1
            FloatShield.actuatorWrite(0)             
            break                                 % Stop program if they do overlap
        end
        nextStep = 1;                             % Enable step flag
    end
end
save response response                            % Save results in response.mat file
plotPIDResponse('response.mat')                   % Plot results from response.mat file