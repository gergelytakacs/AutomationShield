%   TugShield closed-loop PID response example
%
%   PID feedback control of flexi senzor in the TugShield.
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
%   Created by Eva Vargová.
%   Last update: 23.11.2020.

startScript;

TugShield = TugShield;          % Create TugShield object from TugShield class
TugShield.begin('COM5', 'UNO'); % Initialise shield with used Port and Board type
TugShield.calibrate();          % Calibrate TugShield
PID = PID;                        % Create PID object from PID class

Ts = 0.05;                        % Sampling period in seconds
k = 1;                            % Algorithm step counter
nextStep = 0;                     % Algorithm step flag
samplingViolation = 0;            % Sampling violation flag

R = [20, 50, 15, 90, 60, 120, 75, 40, 120, 30]; % Reference trajectory
T = 40;                                     % Section length
n = length(R);

Kp = 0.25;                          % PID Gain
Ti = 5.0;                           % PID Integral time constant
Td = 0.01;                          % PID Derivative time constant
PID.setParameters(Kp, Ti, Td, Ts);  % Feed the constants to PID object 

k = 1;
for i=1:n
    for ii=1:T
        r_l(ii) = R(i);                             % Set reference to the first point in trajectory
        y_l(ii) = TugShield.sensorRead();           % Read flexi position
        u_l(ii) = PID.compute(r_l-y_l,0,180,0,100);     % Compute PID response
        TugShield.actuatorWrite(u_l);                 % Actuate
        k = ((i-1)*T+1):i*T;
        r(k) = r_l(ii);
        y(k) = y_l(ii);
        u(k) = u_l(ii);
        k=k+1;
        pause(Ts)
    end
    k=1;
end

t = 1:0.05:R*T;
disp('The example finished its trajectory.')
plot(t,r)
hold on
plot(t,y)
hold on
plot(t,u)
title('Aplikácia PID regulátora na proces.');
legend('Referenèná hodnota','Odozva snímaèa','Akèný zásah');
xtitle('t(s)');
