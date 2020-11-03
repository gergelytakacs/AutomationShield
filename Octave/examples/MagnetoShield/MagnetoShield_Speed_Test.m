%   MAGNETOSHIELD OCTAVE MINIMAL COMMUNICATION SPEED TEST
%
%   This example tests for the absolute minimal sampling period that is 
%   achievable with this shield using Octave. You will need to install the
%   "arduino" package (see pkg) then the script loads it. Although it is possible to
%   override default I2C speeds, tests show no improvement with increased
%   I2C communication speeds. AVR-based Arduinos, such as the UNO will
%   provide the same speed, as the architecture and clock speeed is
%   identical in all of them. Due or other non-AVR boards compatible with 
%   the MagnetoShield are not supported. The script just tests reading the
%   analog input and sending data to the I2C bus wihtout attempting feedback 
%   control, as these are the most essential operations required for control.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   If you have found any use of this code, please cite our work in your
%   academic publications, such as theses, conference articles or journal
%   papers. A list of publications connected to the AutomationShield
%   project is available at: 
%   https://github.com/gergelytakacs/AutomationShield/wiki/Publications
%
%   Created by:       Gergely Takács
%   Created on:       29.10.2020
%   Last updated by:  Gergely Takács
%   Last update on:   29.10.2020
%   Tested on:        Arduino Uno

clc                                                                 % Clears screen
clear all                                                           % Clears variables


no_samples = 500;                                                   % Desired number of samples
platfrm = 'Uno';                                                    % 'Uno', 'Due', etc.
pkg load arduino                                                    % Load Arduino package

disp('Testing...')                                                  % Message.

A = arduino('COM5',platfrm,'Libraries','I2C');                      % Creates arduino object
configurePin(A,'A3','AnalogInput');                                 % Reads analog yage from Hall sensor

if platfrm == 'Due'                                                % If the platform is a Due
    busno = 1;                                                      % Use I2C bus no. 1
else                                                                % else
    busno = 0;                                                      % use bus no. 0 as default
end

add = scanI2Cbus(A);                                                % Scans I2C
add = str2num(add{1,1});                                            % Decimal address of DAC
DAC = device(A,'I2CAddress',add,'bus',busno,'bitrate',400000);      % Initializes I2c communication
tet = zeros(100,1);                                                 % Pre-allocate memory for execution time measurements
for i = 1:no_samples                                                % Performs several measurements
    tic;                                                            % Starts stopwatch
    y = readVoltage(A,'A3');                                        % Reads voltage from the Hall sensor to variable y
    write(DAC,[64 255],'uint8');                                    % Writes input to DAC
    tet(i,1) = toc;                                                 % Stops stopwatch and notes execution time
    write(DAC,[64 0],'uint8');                                      % Turns off DAC in preparation for next cycle
end

disp('Done.')                                                       % Message.

t = 1:no_samples;                                                   % Vector for axis
plot(t,tet,'o')                                                     % Plot results
grid on;                                                            % Grid
ylabel('Total execution time [s]')                                  % Y axis label
xlabel('Sample (-)')                                                % X axis label
legend('MagnetoShield GNU Octave Communication Speed Test')             % Legend

filename = ['MagnetoShield_Speed_Test_Data_',platfrm];              % Create filename
save(filename,'tet')                                                % Save data