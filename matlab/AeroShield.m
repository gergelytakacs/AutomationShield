%   AeroShield MATLAB API.
%
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Peter Tibensky.
%   Last update: 13.3.2022.

classdef AeroShield < handle

    properties
        arduino;
        as5600;
    end

    properties(Constant)
            AERO_UPIN = 'D5';         % actuator pin
            AERO_RPIN = 'A3';
    end

    methods
        function begin(AeroShieldObject)          % Initialization function
            AeroShieldObject.arduino = arduino();
            AeroShieldObject.as5600 = device(AeroShieldObject.arduino,'I2CAddress',0x36,'bus',1);
            configurePin(AeroShieldObject.arduino,AeroShieldObject.AERO_UPIN, 'DigitalOutput')
            disp('AeroShield initialized.')
        end
        
         function startAngle = calibration(AeroShieldObject)
            write(AeroShieldObject.as5600, 0x0c, 'uint8');  % send command for reading angle
            write(AeroShieldObject.as5600, 0x0d, 'uint8');  % send command for reading angle
            startAngle = read(AeroShieldObject.as5600, 1, 'uint16'); % read answer from sensor
        end

        function PWM = referenceRead(AeroShieldObject)
           PWM= readVoltage(AeroShieldObject.arduino, AeroShieldObject.AERO_RPIN);
        end
        
        function actuatorWrite(AeroShieldObject, PWM)
            writePWMVoltage(AeroShieldObject.arduino, AeroShieldObject.AERO_UPIN, PWM);
        end
       
        
        function RAW = getRawAngle(AeroShieldObject)
            write(AeroShieldObject.as5600, 0x0c, 'uint8');        % send command for reading angle
            write(AeroShieldObject.as5600, 0x0d, 'uint8');        % send command for reading angle
            RAW = read(AeroShieldObject.as5600, 1, 'uint16');     % read answer from sensor
        end
    end
end