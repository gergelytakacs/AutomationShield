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
%   Last update: 17.10.2018.

if ~exist('CI_Testing','var')           % Unless it is a CI test
    clc                                 % Clears screen
end
clear;                                  % Clears all variables                

HeatShield=HeatShield;                  % Construct object from class
HeatShield.begin();                     % Initialize shield

u=40;                                   % [oC] Open-loop reference
Ts=2;                                   % [s] Sampling period
runTime=3600;                           % [s] Total runtime
stepEnable = 0;                         % Algorithm step flag
k=1;                                    % Algorithm step counter

tic                                     % Start measuring time
while(1)                                % Infinite loop
    if (stepEnable)                     % If flag is enabled
        HeatShield.actuatorWrite(u);    % [%] Power 
        y = HeatShield.sensorRead();    % [oC] Temperature
        
        response(k,:)=[u y];            % Store results
        plotLive(response(k,:));        % Live plot
      
        k=k+1;                          % Next sample no.
        stepEnable = 0;                 % Disable step.
    end
    if (toc>=Ts*k)                      % If its time
        stepEnable = 1;                 % Enable the step
    elseif (toc>=runTime)
            break
    end
end

save response response                  % Data file with response
