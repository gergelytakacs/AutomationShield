%   MotoShield PID control example
% 
%   This is an example of MotoShield Matlab API in use featuring
%   two modes of reference value setting. For PID computation, PID class is
%   is used.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács and Ján Boldocký.
clear
close all
clc 

Manual = 0; %-Defining reference Mode # MANUAL / AUTO
MotoShield = MotoShield;        %-Creating MotoShield object
MotoShield.begin('COM3','UNO'); %-Initialize MotoShield
MotoShield.calibration()         %-Calibration
PID = PID;                        %-Creating PID object

Ts = 0.04;                       %-Defining Sample period in seconds
k = 1;                            %-Sample index
stepEnable = 0;                     % Algorithm step flag

R = [80, 70, 50, 85, 35, 60]; %-Reference trajectory
T = 100;                                     %-Section length
i = 0;                                        %-Section counter

Kp = 0.007;                          %-PID constants
Ti = 0.002;                          
Td = 0.1;                          
PID.setParameters(Kp, Ti, Td, Ts);  %-Setting PID constants

tic                                          %-Start measuring time
while (1)     %-Loop
        if (toc >= Ts * k)                            %-If its time for next sample
            stepEnable = 1;                             %-Next step premission
        end
    if (stepEnable)                          %-Start algorithm if stepEnabled
        if(Manual == 0)             %-AUTOMATIC mode 
        if (mod(k, T*i) == 1)                %-Moving trough trajectory values
            i = i + 1;                       
            if (i > length(R))               %-If trajectory ended
                MotoShield.actuatorWrite(0.0); %-Disable Motor
                break                        %-End of Loop
            end
                r = R(i);                 %-Move to next the value in trajectory
            end                    
        end
        if(Manual == 1)                 %-MANUAL mode
            r = MotoShield.referenceRead(); %-Read reference from potentiometer
        end
        y = MotoShield.sensorRead();       %-Sense RPM
        u = PID.compute(r-y, 0, 100, 0, 100);  %-computing Actuating Signal
        MotoShield.actuatorWrite(u);          %-Actuate
        response(k, :) = [r, y, u];            %-Storing data
        k = k + 1;                             %-Incrementing sample index
        stepEnable = 0;                          %-Disabling step flag
    end                                        %-End of Algorithm
end
save response response                            %-Saving data in response.mat file
disp('The example finished its trajectory. Results have been saved to "response.mat" file.')
plotPIDResponse('response.mat')                 %-Plot data from response.mat
