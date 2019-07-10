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
%   Last update: 10.7.2019.

classdef HeatShield < handle

    properties
        arduino;
    end

    properties(Constant)
        HEAT_YPIN = 'A0';         % NTC Thermistor (Sensor)
        HEAT_UPIN = 'D3';         % Cartridge (Actuator)
        REF_TEMP = 25.0 + 273.15; % Thermistor reference temperature
        ABSZERO = 273.15;         % Absolute zero
        NTC_RES = 100000.0;       % Resistance of the thermistor
        VD_RES = 100000.0;        % Resistance of voltage divider arm
        VD_REF = 5.0;             % Input for the voltage divider
        NTC_BETA = 3950.0;        % value of Beta factor from datasheet MF58
    end

    methods
        function begin(HeatShieldObject)          % Initialization function
            HeatShieldObject.arduino = arduino();
            disp('HeatShield initialized.')
        end

        function actuatorWrite(HeatShieldObject, percent)
            writePWMDutyCycle(HeatShieldObject.arduino, HeatShieldObject.HEAT_UPIN, (percent / 100));
        end

        function U = getThermistorVoltage(HeatShieldObject)
            U = readVoltage(HeatShieldObject.arduino, HeatShieldObject.HEAT_YPIN);
        end

        function R = getThermistorResistance(HeatShieldObject)
            Vterm = HeatShieldObject.getThermistorVoltage();
            R = ((Vterm * HeatShieldObject.VD_RES) / (HeatShieldObject.VD_REF - Vterm));
        end

        function y = sensorRead(HeatShieldObject)
            y = (1 / ((1 / HeatShieldObject.REF_TEMP) + (log(HeatShieldObject.getThermistorResistance()/HeatShieldObject.NTC_RES) / HeatShieldObject.NTC_BETA))) - HeatShieldObject.ABSZERO;
        end
    end
end
