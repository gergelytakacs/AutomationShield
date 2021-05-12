%   TugShield MATLAB API.
%
%   This script contains TugShield class definition including
%   functions for TugShield initialisation, calibration, 
%   acquisition of measured distances and of potentiometer runner
%   bending. Purpose of this class is to make the TugShield
%   operation in MATLAB more effective. 
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Eva Vargová.
%   Last update: 15.11.2020.

classdef TugShield < handle
 
    properties(Access = public)           % Public class objects
        arduino;                          % Object for arduino board
        servo;                            % Object for servo motor library
        was_calibrated = false;           % Calibration flag
    end

    properties(Constant)
        TUG_YPIN = 'A0';            % Pin of flexi senzor
        TUG_UPIN = 'D10';           % Pin of servo motor (Actuator)
        K_MIN = 675;                
        K_MAX = 0;                  
        DELAY_VALUE = 0.5;          
        SERVO_MAX_FLEX = 0;         % Maximal Servo position 180 degrees
        SERVO_MIN_FLEX = 1;       % Maximal Servo position 0 degrees
        DEFAULT_MIN = 500.0;
        DEFAULT_MAX = 1000.0;
    end

    methods
        function begin(TugShieldObject,aPort, aBoard)                                     % Initialization function
            listOfLibraries = listArduinoLibraries();                       % List of available arduino libraries
            libraryIsPresent = sum(ismember(listOfLibraries,'Servo'));      % Check if required library is present
            if ~libraryIsPresent                                            % If not, throw an error
                error('Servo library was not detected by MATLAB! Use listArduinoLibraries() command to verify its presence.')                
            end
            TugShieldObject.arduino = arduino(aPort,aBoard, 'Libraries', 'Servo');                 % Initialising object for arduino board
            TugShieldObject.servo = servo(TugShieldObject.arduino,TugShieldObject.TUG_UPIN);        % Initialising object for servo
            writePosition(TugShieldObject.servo,TugShieldObject.SERVO_MIN_FLEX);                    % Set the servo position to zero
            disp('TugShield initialized.')
        end
        
        function calibrate(TugShieldObject)                                 % Calibration function
            disp('TugShield calibrated.')
            writePosition(TugShieldObject.servo,TugShieldObject.SERVO_MIN_FLEX);  % Set servo position to minimum
            pause(TugShieldObject.DELAY_VALUE)
            for i=1:10
                sensorRead=readVoltage(TugShieldObject.arduino,TugShieldObject.TUG_YPIN)			% Read value from flexi sensor
                if i==1
                    if(sensorRead<TugShieldObject.K_MIN)					% Comparison of measured value with determined value		
                        k_minimal=sensorRead;                               % Value assignment
                    end
		     		pause(TugShieldObject.DELAY_VALUE)
                else
                    if(sensorRead<k_minimal)                                 % Comparison of k_minimal with the measured value		
                        k_minimal=sensorRead;                                % Value assignment
                    end
                end
            end
            writePosition(TugShieldObject.servo,TugShieldObject.SERVO_MAX_FLEX);  % Set servo position to maximum
            pause(TugShieldObject.DELAY_VALUE)
            for i=1:10
                sensorRead=readVoltage(TugShieldObject.arduino,TugShieldObject.TUG_YPIN);			% Read value from flexi sensor
                if i==1
                    if(sensorRead>TugShieldObject.K_MAX)					% Comparison of measured value with determined value		
                        k_maximum=sensorRead;                               % Value assignment
                    end
		     		pause(TugShieldObject.DELAY_VALUE)
                else
                    if(sensorRead>k_maximum)                                 % Comparison of k_maximum with the measured value		
		                k_maximum=sensorRead;                                % Value assignment
                    end
                end
            end
             was_calibrated = true;
             disp('TugShield calibrated.')
        end
        
        function actuatorWrite(TugShieldObject,angle)
            right_angle = 1 - angle;                           % Count real angle of servo
            writePosition(TugShieldObject.servo,right_angle);   % Set real angle to servo 
        end

        function y = sensorRead(TugShieldObject)
            sensorRead = readVoltage(TugShieldObject.arduino,TugShieldObject.TUG_YPIN);	% read senzor value
            if was_calibrated == true                                                   % Remap in case the calibration has been done
                y = interp1([k_minimal,k_maximal],[0.00,100.00],sensorRead); 
            else                                                                        % Remap in case the calibration has not been done
                y = interp1([TugShieldObject.DEFAULT_MIN,TugShieldObject.DEFAULT_MAX],[0.00,100.00],sensorRead); 
            end
        end
    end
end
