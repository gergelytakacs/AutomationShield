%   OptoShield system identification experiment
% 
%   Acquires data for system identification and plots
%   the data.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Tak�cs. 
%   Last update: 01.10.2018.

resultID=readExperiment(7*200,'COM22',9600);
save resultID resultID

plot(resultID)
hold on
legend('Output','Input')
xlabel('Sample no. (-)')
ylabel('Input, output level (%)')
grid on

