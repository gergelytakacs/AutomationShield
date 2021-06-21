%   HeatShield input-output interface
% 
%   Demonstrates the input-output functionality of the
%   HeatShield MATLAB API.
%
%   The script initializes the board, then reads the 
%   output, writes to the input and shows how alternative
%   readings from the termocouple work.
%  
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 17.10.2018.
  
startScript;                                    % Clears screen and variables, except allows CI testing

HeatShield=HeatShield;  % Construct object from class
HeatShield.begin();     % Initialize shield

% Input (thermistor)
y = HeatShield.sensorRead()              % [oC] Temperature

% Output (cartridge)
u=0;                                     % [%] Power
HeatShield.actuatorWrite(u)              % Set power

% Alternative input (thermistor)
V = HeatShield.getThermistorVoltage()    % Voltage
R = HeatShield.getThermistorResistance() % Resistance