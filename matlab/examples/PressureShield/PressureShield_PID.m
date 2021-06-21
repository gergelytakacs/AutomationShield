%   PressureShield simple PID example
% 
%   Performs a discrete PID control of the PressureShield
%   hardware with input saturation.
%
%   This example initializes the PressureShield device, then
%   requests a step change of reference to 50 hPa. 
%   The input to the pump is calculated by an absolute
%   discrete PID controller in its ideal form. The inputs are saturated,
%   no integral windup handling is provided. The example plots the live
%   response. When the experiment is over, the data is saved to a file.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Martin Staron.
%   Last update: 26.4.2021.
  
startScript;                          

PressureShield = PressureShield;            % Construct object from class
PressureShield.begin();                     % Initialize shield

PID = PID;

Kp = 0.45;                                     % PID Gain
Ti = 0.25;                                     % PID Integral time constant
Td = 0.01;                                  % PID Derivative time constant
                                   
R=[80 50 70 40 90 30 60 20 40 70];       % [HPa] Overpressure closed-loop reference
Ts = 0.2;                                   % [s] Sampling period
runTime = 1000;                              % [s] Total runtime

secLength = 100;         % Length of a reference trajectory section
stepEnable = 0;                             % Algorithm step flag
k = 1;                                      % Algorithm step counter
j = 1;                                      % Reference section counter
r = R(1);                                   % First reference
y = 0;                                      % Output initialize
initPressure = PressureShield.sensorRead();         % Read reference pressure
PID.setParameters(Kp, Ti, Td, Ts);  % Feed the constants to PID object 

tic                                               % Start measuring time
while(1)

    % Infinite loop
    if (stepEnable)                       % If flag is enabled
     % Read the output
        y = (PressureShield.sensorRead()-initPressure)/100;  % [HPa] OverPressure
     % Pick reference
        if (mod(k,secLength*j)==0);         % If time for new reference 
            j=j+1;                          % Next reference section
            if (j == 12)
                j = 1;
            end
            r=R(j);                         % Pick new reference    
        end
%      r = PressureShield.referenceRead();   
     % Compute PID

       u = PID.compute(r-y, 0, 100, 0, 100);  % Compute PID response 
     % Write to hardware
        PressureShield.actuatorWrite(u);    % [%] Power
        response(k,:) = [r y u];            % Store results
%         plotLive(response(k,:));            % Live plot
      
        k = k+1;                            % Next sample no.
        stepEnable = 0;                     % Disable step.
    end                                     % Step end
    if (k<runTime)                          % If its time
        stepEnable = 1;
        % Enable the step
    elseif (k>=runTime)                   % Experiment over
        PressureShield.actuatorWrite(0);    % Input off
        break                               % Exit while
    end
end
toc
save response response                      % Data file with response
plot(response)