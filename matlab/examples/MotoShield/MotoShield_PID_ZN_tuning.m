%   MotoShield PID control example using Ziegler-Nichols tuning method.
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
%   Created by Ján Boldocký.

startScript;                                    % Clears screen and variables, except allows CI testing

Manual = 0; %-Defining reference Mode # MANUAL / AUTO
MotoShield = MotoShield;        %-Creating MotoShield object
MotoShield.begin('COM3','UNO'); %-Initialize MotoShield
%MotoShield.calibration()         %-Calibration
PID = PID;                        %-Creating PID object

Ts = 0.02;                       %-Defining Sample period in seconds
k = 1;                            %-Sample index
stepEnable = 0;                     % Algorithm step flag

R = [1200.0,800.0,1200.0,1000.0,700.0,1000.0,1200.0,600.0,1100.0,500.0,0.0]; %-Reference trajectory
T = 20;                                     %-Section length
i = 0;                                        %-Section counter

Kp = 0.00127;                          %-PID constants
Ti = 0.022;                          
Td = 0.0055;                          
PID.setParameters(Kp, Ti, Td, Ts);  %-Setting PID constants

tic                                          %-Start measuring time
while (1)     %-Loop
        if (toc >= Ts * k)                            %-If its time for next sample
            stepEnable = 1;                             %-Next step permission
        end
    if (stepEnable)                          %-Start algorithm if stepEnabled
        if(Manual == 0)             %-AUTOMATIC mode 
        if (mod(k, T*i) == 1)                %-Moving through trajectory values
            i = i + 1;                       
            if (i > length(R))               %-If trajectory ended
                MotoShield.actuatorWriteVolt(0.0); %-Disable Motor
                break                        %-End of Loop
            end
                r = R(i);                 %-Move to next the value in trajectory
            end                    
        end
        if(Manual == 1)                 %-MANUAL mode
            r = MotoShield.referenceRead(); %-Read reference from potentiometer
        end
        y = MotoShield.sensorReadRadian()       %-Sense RPM
        u = PID.compute(r-y, 0, 5, 0, 5);  %-computing Actuating Signal
        MotoShield.actuatorWriteVolt(constrain(u,0,5));          %-Actuate
        response(k, :) = [r, y, u];            %-Storing data
        k = k + 1;                             %-Incrementing sample index
        stepEnable = 0;                          %-Disabling step flag
    end                                        %-End of Algorithm
end
save response response                            %-Saving data in response.mat file
disp('The example finished its trajectory. Results have been saved to "response.mat" file.')
plotResults('response.mat')                 %-Plot data from response.mat