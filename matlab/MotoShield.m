%   MotoShield R1 MATLAB API.
%
%   This file contains MotoShield API which includes class
%   and methods for full control of the prototype (R1). Beware,
%   digital pins 2 and 4 have to be connected with a jumper cable
%   in order for Velocity Measurement method to work.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Ján Boldocký
%   Last update: 19.04.2020
classdef MotoShield < handle %-class definition

    properties(Access = public)%-public entities
        arduino; %-object for the MCU
        encoder; %-object for quadrature incremental encoder
        %-calibration variables
        minDuty = 25; %-default value
        minRPM;
        maxRPM;
    end

    properties(Constant) %-defining constants
        MOTO_YPIN1 = 'D2'; %-encoder channelA
        MOTO_YPIN2 = 'D3'; %-encoder channelB
        MOTO_UPIN  = 'D5'; %-PWM input for motor
        MOTO_DIR_PIN1 = 'D6'; %-polarity H-bridge pin1
        MOTO_DIR_PIN2 = 'D7'; %-polarity H-bridge pin2
        MOTO_YV1  = 'A0';   %-Voltage before shunt resistor
        MOTO_YV2  = 'A1';   %-Voltage after shunt resistor
        MOTO_YAMP1 = 'A2';  %-Differential OpAmp output
        MOTO_YAMP2 = 'A3';  %-Non-inverting OpAmp output
        MOTO_RPIN = 'A4';   %-potentiometer reference
        SHUNT = 10.0;         %-resistance of shunt resistor in ohms
        AMP_GAIN = 2.96;    %-gain of non-inverting OpAmp
        PPR = 7;            %-pulses per rotation
    end

    methods(Access = public) 
        function begin(MotoShieldObject, Port, Board) %-initialization
            MotoShieldObject.arduino = arduino(Port,Board,'Libraries','rotaryEncoder'); %-initialize MCU
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YPIN1,'Interrupt'); %-setting correct pin modes
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YPIN2,'Interrupt');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_RPIN,'AnalogInput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YV1,'AnalogInput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YV2,'AnalogInput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YAMP1,'AnalogInput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YAMP2,'AnalogInput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_UPIN,'PWM');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN1,'DigitalOutput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN2,'DigitalOutput');
            MotoShieldObject.encoder = rotaryEncoder(MotoShieldObject.arduino,MotoShieldObject.MOTO_YPIN1,MotoShieldObject.MOTO_YPIN2,MotoShieldObject.PPR); %-initialize encoder for pins D2 and D3
            setDirection(MotoShieldObject); %-making setDirection() function non-mandatory
            disp('MotoShield initialized.') %-print
        end        
        function setDirection(MotoShieldObject, direction) %-determinate direction of rotation
           if nargin < 2 %-if unspecified direction
               direction = 1; %-default value = true
           end
           if direction %-clockwise
               writeDigitalPin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN1,0);
               writeDigitalPin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN2,1);
           else %-counterclockwise
               writeDigitalPin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN1,1);
               writeDigitalPin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN2,0);
           end
        end
        function actuatorWrite(MotoShieldObject, percentValue) %-actuate in percent
            if (percentValue < MotoShieldObject.minDuty) && (percentValue ~= 0) %-if input value is 0, motor turns off
                percentValue = MotoShieldObject.minDuty; %-if input value is lesser than minimal Duty, make it minimal
            end
            writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN, (percentValue / 100));
        end
        function ref = referenceRead(MotoShieldObject) %-returns reference of potentiometer in percent
           ref = readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_RPIN) * 100 / 5;
        end
        function voltageDrop = sensorReadVoltage(MotoShieldObject) %-voltageDrop across the shunt resistor in V
            voltageDrop = readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_YV1)-readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_YV2); 
        end
        function voltageDropAmp1 = sensorReadVoltageAmp1(MotoShieldObject) %-output from differential OpAmp in V
            voltageDropAmp1 = readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_YAMP1)*100/5;
        end
        function voltageDropAmp2 = sensorReadVoltageAmp2(MotoShieldObject) %-output from non-inverting OpAmp in V (not amplified value)
            voltageDropAmp2 = readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_YAMP2)/MotoShieldObject.AMP_GAIN * 100 / 5;
        end
        function current = sensorReadCurrent(MotoShieldObject) %-Ohms law # current calculation
            current = voltageDropAmp2 / MotoShieldObject.SHUNT * 1000.0;
        end
        function calibration(MotoShieldObject) %-calibration
        actuatorWrite(MotoShieldObject,100);   %-actuate
        pause(1);   %-wait for motor to spin at maxRPM
        MotoShieldObject.maxRPM = readSpeed(MotoShieldObject.encoder);
        actuatorWrite(MotoShieldObject,0);%-stop the motor
        pause(0.5); %-wait for motor to stop spinning
        i = 15; %-initial duty in %, presumed potentionally minimal
        while(1) %-infinite loop
           resetCount(MotoShieldObject.encoder); %-resets count to 0
            actuatorWrite(MotoShieldObject,i); %-actuate
            pause(0.3); %-wait for at least one revolution           
            if abs(readCount(MotoShieldObject.encoder)) > 8 %-detecting motion
                pause(1); %-%-wait for motor to spin at minRPM
                MotoShieldObject.minDuty = i; %-if motor is spinning, this is minimal duty
                MotoShieldObject.minRPM = readSpeed(MotoShieldObject.encoder); %-sense minimal RPM
                actuatorWrite(MotoShieldObject,0); %-turn off motor
                break; %-exit the loop
            end
            i=i+1; %-increment
        end %-end of while loop
        end %- end of funciton
        function rpm = sensorReadRPM(MotoShieldObject) %-sensing RPM # proper nomenclature
        rpm = readSpeed(MotoShieldObject.encoder);
        end
        function rpmPerc = sensorReadRPMPerc(MotoShieldObject) %-sensing RPM in percents # constrained
        rpmPerc = constrain(map(readSpeed(MotoShieldObject.encoder),MotoShieldObject.minRPM,MotoShieldObject.maxRPM,0,100),0,100);
        end
    end
end
