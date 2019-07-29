%   HeatShield simple PID example
% 
%   Performs a discrete PID control of the HeatShield
%   hardware with input saturation.
%
%   This example initializes the HeatShield device, then
%   requests a step change of reference to 40 C. A pre/determined
%   reference trajectory is divided according to total experiment
%   length. The input to the cartridge is caluclated by an absolute
%   discrete PID controller in its ideal form. The inputs are saturated,
%   no integral windup handling is provided. The example plots the live
%   response. When the experiment is over, the data is saved to a file.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 18.07.2019.
  
clc; clear all;                             % Clears screen and all variables

HeatShield=HeatShield;                      % Construct object from class
HeatShield.begin();                         % Initialize shield

Kp=3.0;                                     % PID Gain
Ti=30.0;                                    % PID Integral time constant
Td=1.0;                                     % PID Derivative time constant
umax=100;                                   % Maximum input
umin=0;                                     % Minimum input

R=[40 60 50 35];                            % [oC] Closed-loop reference
Ts=2;                                       % [s] Sampling period
runTime=7200;                               % [s] Total runtime

secLength=(runTime/Ts)/length(R);           % Length of a reference trajectory section
stepEnable = 0;                             % Algorithm step flag
k=1;                                        % Algorithm step counter
j=1;                                        % Reference section counter
r=R(1);                                     % First reference
eSum=0;                                     % Error sum
ep=0;                                       % Previous error
y=0;                                        % Output initialize

tic                                         % Start measuring time
while(1)                                    % Infinite loop
    if (stepEnable)                         % If flag is enabled
     % Read the output
        y = HeatShield.sensorRead();        % [oC] Temperature
        
     % Pick reference
        if (mod(k,secLength*j)==0);         % If time for new reference 
            j=j+1;                          % Next reference section
            r=R(j);                         % Pick new reference    
        end
        
     % Compute PID
        e = r-y;                            % [oC] Reference
        eSum=eSum+e;                        % Integral
        eSum=constrain((Kp*Ts/Ti)*eSum,umin,umax)/(Kp*Ts/Ti);  % Integral windup clamp
        u =Kp*(e+(Ts/Ti)*eSum+(Td/Ts)*(e-ep)); % PID 
        ep = e;                             % Data store
        u=constrain(u,umin,umax);           % Input saturation
        
     % Write to hardware
        HeatShield.actuatorWrite(u);        % [%] Power

        response(k,:)=[r y u];              % Store results
        plotLive(response(k,:));            % Live plot
      
        k=k+1;                              % Next sample no.
        stepEnable = 0;                     % Disable step.
    end                                     % Step end
    if (toc>=Ts*k)                          % If its time
        stepEnable = 1;                     % Enable the step
    elseif (toc>=runTime)                   % Experiment over
        HeatShield.actuatorWrite(0);        % Input off
        break                               % Exit while
    end
end

save response response                      % Data file with response
