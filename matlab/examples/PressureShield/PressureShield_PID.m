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

Kp = 0.9;                                     % PID Gain
Ti = 1;                                     % PID Integral time constant
Td = 0.1;                                  % PID Derivative time constant
                                   
umin = 0;                                   % Minimum input
umax = 100;                                 % Maximum input

R=[60 40 70 50 80];                         % [HPa] Overpressure closed-loop reference
Ts = 0.1;                                     % [s] Sampling period
runTime = 75;                               % [s] Total runtime

secLength = 100;         % Length of a reference trajectory section
stepEnable = 0;                             % Algorithm step flag
k = 1;                                      % Algorithm step counter
j = 1;                                      % Reference section counter
r = R(1);                                   % First reference
eSum = 0;                                   % Error sum
ep = 0;                                     % Previous error
y = 0;                                      % Output initialize
init = PressureShield.sensorRead();         % Read reference pressure

tic                                                  % Start measuring time
while(1)                                             % Infinite loop
    if (stepEnable)&&(toc>5.0)                       % If flag is enabled
     % Read the output
        y = (PressureShield.sensorRead()-init)/100;  % [HPa] OverPressure
        
     % Pick reference
        if (mod(k,secLength*j)==0);         % If time for new reference 
            j=j+1;                          % Next reference section
            if (j == 6)
                j = 1;
            end
            r=R(j);                         % Pick new reference    
        end
        
     % Compute PID
        e = r-y;                                                 % [HPa] Reference
        eSum = eSum+e;                                          % Integral
        eSum = constrain((Kp*Ts/Ti)*eSum,umin,umax)/(Kp*Ts/Ti);  
        u = double(Kp*(e+(Ts/Ti)*eSum+(Td/Ts)*(e-ep))); % PID 
        ep = e;                                                 % Data store
        u = constrain(u,umin,umax);                              % Input saturation
        
     % Write to hardware
        PressureShield.actuatorWrite(u);    % [%] Power
        response(k,:) = [r y u];            % Store results
        %plotLive(response(k,:));            % Live plot
      
        k = k+1;                            % Next sample no.
        stepEnable = 0;                     % Disable step.
    end                                     % Step end
    if (toc<runTime)                          % If its time
        stepEnable = 1;                     % Enable the step
    elseif (toc>=runTime)                   % Experiment over
        PressureShield.actuatorWrite(0);    % Input off
        break                               % Exit while
    end
end

save response response                      % Data file with response
plot(response)