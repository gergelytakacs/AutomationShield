%   TugShield system identification experiment
%   Creates a pulse train to extract input-output data.
  
%   This example initializes the sampling subsystem from the 
%   AutomationShield library and starts an open-loop experiment
%   with the intention of extracting useful data for a system
%   identification procedure. 
  
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Eva Vargová.
%   Last update: 16.5.2021.

startScript;   

TugShield = TugShield;          % Create TugShield object from TugShield class
TugShield.begin('COM3', 'UNO'); % Initialise shield with used Port and Board type
TugShield.calibrate();          % Calibrate TugShield

disp('Flex sensor test...servo position: 0 ')
TugShield.actuatorWrite(0);
pause(1);
y = TugShield.sensorRead();
if (y >= 0.0 && y <= 15.0)
    disp('The values of flex sensor are OK');
    disp(y);
else 
    disp('The values of flex sensor are NOK');
    disp(y);
end
pause(0.5);

disp('Flex sensor test...servo position: 180')
TugShield.actuatorWrite(180);
pause(1);
y = TugShield.sensorRead();
if (y >= 85.0 && y <= 100.0) 
    disp('The values of flex sensor are OK');
    disp(y);
else 
    disp('The values of flex sensor are NOK');
    disp(y);
end

disp('Test completed')