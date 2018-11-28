%   HeatShield system identification example
% 
%   Measures and logs the printer head temperature for
%   system identification purposes.
%
%   This example initializes the HeatShield device, then
%   supplies a step change of input to 40% power to the
%   heating cartridge and plots the response. When the
%   experiment is over, the data is saved to a file.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 18.11.2018.
  
clc; clear all;                              % Clears screen and all variables
resultID=readExperiment(5*1000,'COM22',2000000);
save resultID resultID

plot(resultID)
hold on
legend('Input','Injected Noise','Output')
xlabel('Sample no. (-)')
ylabel('Input, output level (%)')
grid on
  