% clear all objects and variables
clear all
clc 

% initialisation of AeroShield library 
AeroShield=AeroShield; 

% create an arduino object + hall sensor object 
AeroShield.begin();

% calibration
startangle= AeroShield.calibration(); 
lastangle=startangle+1024; % save startangle value + 1024(90°).(for mapping purpose)

% initialisation of PID library 
PID = PID;

Ts = 0.002;             % sampling period           
Kp=0.035 ;              % PID Gain
Ti=0.00038 ;            % PID Integral time constant
Td=0.0000025 ;          % PID Derivative time constant
PID.setParameters(Kp, Ti, Td, Ts); % set parameters in PID library 

MANUAL= 0;              % manual reference using potentiometer (1)  automatic reference trajectory (0) 
R=[23 48 70 12 40 19 9 43 80];     % closed-loop reference in percent
secLength=25;           % length of a reference trajectory section, measured in samples
stepEnable = 0;         % bool value for step enabling
k=1;                    % sample/step counter
j=1;                    % counter for reference trajectory
r=R(1);                 % first reference trajectory value
y=0;                    % output value (hall sensor value)

tic                     % MATLAB time measure function
while(1)                % infinite loop, until "break" command is used 
    if (stepEnable)     % if next step is enable
        
     % Reading output value
        RAW = AeroShield.getRawAngle();   % get raw angle value
        y = map(RAW, startangle, lastangle, 0.0, 100.0); % map raw value to percent 
        
     % Pick reference
        if MANUAL                               % if Manual mode is active
            PWMvalue = AeroShield.referenceRead();          % read reference from potentiometer
            r=map(PWMvalue, 0, 5, 0, 100);               % map value to percent
        else 
            if (mod(k,secLength*j)==0);             % if is time for new reference 
            j=j+1;                                  % go to next reference
                if (j > length(R))                  % if at the end of trajectory
                 AeroShield.actuatorWrite(0.0);     % stop the motor
                break                               % stop the program execution
                end
            r=R(j);                                 % pick new reference    
            end
        end
   
     % compute PID: Error, SaturationMin, SaturationMax, AntiWindupMin, AntiWindupMax
        u = PID.compute(r-y, 0, 80, 0, 80); 
        
     % actuate
         coercedInput = constrain(u, 0, 100);       % safety precaution 
         PWM=remappedValue(coercedInput, 0, 100, 0, 5);    % remap from percent to voltage value
         AeroShield.actuatorWrite(PWM);             % actuate

        PIDresponse(k,:)=[r y u];           % store data
        plotLive(PIDresponse(k,:));         % plot data
        k=k+1;                              % go to next sample
        stepEnable = 0;                     % disable step 
    end                                     % step end
    
    if (toc>=Ts*k)                          % if it is time for next sample
        stepEnable = 1;                     % enable next step
    end
    
    if (y > 110)                            % if angle of pendulum bigger than 110°
        AeroShield.actuatorWrite(0.0);      % stop the motor 
        disp('Angle of pendulum too high. AeroShield is turned off')
        break                               % stop the program
    end
end    % while loop end

disp('Example finished. Captured data saved to "PIDresponse.mat" file.')
save PIDresponse PIDresponse                % save data file