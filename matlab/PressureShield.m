%   PressureShield MATLAB API.
%
%   The script initializes the board, then reads the
%   output, writes to the input and reads reference
%   from the potentiometer.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Martin Staron.
%   Last update: 4.5.2021.

classdef PressureShield < handle

    properties(Access = public) 
        arduino;
        dev;
    end

    properties(Constant)
        Pressure_RPIN = 'A0';          % BMP280 (Sensor)
        Pressure_UPIN = 'D11';         % Pump (Actuator)
    end

    methods
        function begin(PressureShieldObject)        
            PressureShieldObject.arduino = arduino();
            PressureShieldObject.dev = device(PressureShieldObject.arduino,'I2CAddress','0x76');
            BMP280Init(PressureShieldObject.dev);
            disp('PressureShield initialized.')
        end
        
        function actuatorWrite(PressureShieldObject, percent)
            input = single(percent);
            writePWMDutyCycle(PressureShieldObject.arduino, PressureShieldObject.Pressure_UPIN, (input/100));
        end
        
        function reference = referenceRead(PressureShieldObject)
            reference = readVoltage(PressureShieldObject.arduino,PressureShieldObject.Pressure_RPIN) * 100 / 5;
        end
        
        function y = sensorRead(PressureShieldObject)
            y = BMP280(PressureShieldObject.dev);       
        end

    end
end