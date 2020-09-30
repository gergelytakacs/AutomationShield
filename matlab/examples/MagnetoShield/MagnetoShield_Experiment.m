%   MagnetoShield data acquisition experiment
% 
%   This example logs a measurement from the MagnetoShield device. 
%   Please load a code containing feedback control or a valid 
%   identification before proceeding with this script. Note that you
%   may need to check the communication port and speed, or correct
%   the expected experiment length.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takacs. 
%   Last update: 25.09.2019.


l=1000;                         % Experiment section length
lngth=5*l;                      % Number of sections
port='COM4'                    % Your port
baud=2000000;                    % Baud rate (2M for AVR, 250 000 for SAM3)

result=readExperiment(lngth,port,baud);
plot(result)                    % Just a preview
save result result              % Save in file