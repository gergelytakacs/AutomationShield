%   HeatShield MATLAB API.
%
%   The script initializes the board, then reads the 
%   output, writes to the input and shows how alternative
%   readings from the termocouple work.
%  
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 17.10.2018.

classdef HeatShield < handle
    
    properties
        arduino;
    end
    
    properties (Constant)
        HEAT_YPIN = 'A0';          % NTC Thermistor (Sensor)
        HEAT_UPIN ='D3';           % Cartridge (Actuator)
        REF_TEMP=25.0 + 273.15;    % Thermistor reference temperature
        ABSZERO =273.15;           % Absolute zero
        NTC_RES=100000.0;          % Resistance of the thermistor
        VD_RES=100000.0;           % Resistance of voltage divider arm
        VD_REF=5.0;                % Input for the voltage divider
        NTC_BETA=3950.0;           % value of Beta factor from datasheet MF58
    end
    
    methods     
        function begin(obj)           % Initialization function
            obj.arduino=arduino();
            disp('HeatShield initialized.')
        end
        
        function actuatorWrite(obj,percent)
            writePWMDutyCycle(obj.arduino, obj.HEAT_UPIN, (percent/100));
        end
        
        function U=getThermistorVoltage(obj)
                U=readVoltage(obj.arduino, obj.HEAT_YPIN);
        end
        
        function R=getThermistorResistance(obj)
                Vterm = obj.getThermistorVoltage();
                R=((Vterm*obj.VD_RES)/(obj.VD_REF-Vterm));
        end
        
        
        function y=sensorRead(obj)
                y=(1 / ((1 / obj.REF_TEMP) + (log(obj.getThermistorResistance() / obj.NTC_RES) / obj.NTC_BETA))) - obj.ABSZERO;    
        end
    end
end
