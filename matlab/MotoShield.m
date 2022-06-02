%   MotoShield R1 and R2 MATLAB API.
%
%   This file contains MotoShield API which includes class
%   and methods for full control of the prototype. Beware, when using R1,
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
        direction = true; % default clockwise
        SHIELDRELEASE = 'R2'; % Default latest
        VREF;
        %Pin number declaration
        MOTO_YPIN1 = 'D2'; %-encoder channelA
        MOTO_YPIN2 = 'D3'; %-encoder channelB
        MOTO_RPIN = 'A0'; % Reference Read        
        %MotoShield R1 Pins
        MOTO_UPIN  = 'D5'; %-PWM input for motor
        MOTO_DIR_PIN1 = 'D6'; %-polarity H-bridge pin1
        MOTO_DIR_PIN2 = 'D7'; %-polarity H-bridge pin2
        MOTO_YV1  = 'A0';   %-Voltage before shunt resistor
        MOTO_YV2  = 'A1';   %-Voltage after shunt resistor
        MOTO_YAMP1 = 'A2';  %-Differential OpAmp output
        %MotoShield R2 Pins
        MOTO_UPIN1 = 'D5';  
        MOTO_UPIN2 = 'D6';
        MOTO_YPIN = 'A3'; % Current Measurement
    end

    properties(Constant) %-defining constants
        PPR = 7;           %-pulses per rotation
        SHUNT = 10.0;         %-resistance of shunt resistor in ohms
        AMP_GAIN = 2.96;    %-gain of non-inverting OpAmp MotoShield R1
    end

    methods(Access = public)
        function MotoShieldObject = MotoShield(release) % Constructor
            if nargin < 1 % if ShieldRelease not specified
                MotoShieldObject.SHIELDRELEASE='R2'; % Latest is default
            else
                MotoShieldObject.SHIELDRELEASE=release;
            end
            if ~(strcmp(MotoShieldObject.SHIELDRELEASE,'R1') || strcmp(MotoShieldObject.SHIELDRELEASE,'R2'))
                error("Availible versions of MotoShield are 'R1' and 'R2'")
            end
            switch MotoShieldObject.SHIELDRELEASE
                case 'R1'
                    MotoShieldObject.MOTO_RPIN = 'A4';
                    MotoShieldObject.VREF=5;
                case 'R2'
                    MotoShieldObject.MOTO_RPIN = 'A0';
                    MotoShieldObject.VREF=3.3;
            end
        end
        function begin(MotoShieldObject, Port, Board) %-initialization
          switch MotoShieldObject.SHIELDRELEASE
              case 'R1'
            MotoShieldObject.arduino = arduino(Port,Board,'Libraries','rotaryEncoder'); %-initialize MCU
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YV1,'AnalogInput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YV2,'AnalogInput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YAMP1,'AnalogInput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YAMP2,'AnalogInput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_UPIN,'PWM');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN1,'DigitalOutput');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN2,'DigitalOutput');          
              case 'R2'
            MotoShieldObject.arduino = arduino(Port,Board,'Libraries','rotaryEncoder','AnalogReferenceMode','external','AnalogReference',3.3); %-initialize MCU
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_UPIN1,'PWM'); %-setting correct pin modes
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_UPIN2,'PWM');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YPIN,'AnalogInput');
          end
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YPIN1,'Interrupt'); %-setting correct pin modes
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_YPIN2,'Interrupt');
            configurePin(MotoShieldObject.arduino,MotoShieldObject.MOTO_RPIN,'AnalogInput');
            MotoShieldObject.encoder = rotaryEncoder(MotoShieldObject.arduino,MotoShieldObject.MOTO_YPIN1,MotoShieldObject.MOTO_YPIN2,MotoShieldObject.PPR); %-initialize encoder for pins D2 and D3
            setDirection(MotoShieldObject); %-making setDirection() function non-mandatory
            disp('MotoShield initialized.') %-print
        end        
        function setDirection(MotoShieldObject, direction) %-determinate direction of rotation
           if nargin < 2 %-if unspecified direction
               direction = true; %-default value = true
           end
           switch MotoShieldObject.SHIELDRELEASE
            case 'R1' 
                 if direction %-clockwise
                     writeDigitalPin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN1,0);
                     writeDigitalPin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN2,1);
                 else %-counterclockwise
                     writeDigitalPin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN1,1);
                     writeDigitalPin(MotoShieldObject.arduino,MotoShieldObject.MOTO_DIR_PIN2,0);
                 end
            case 'R2'
                MotoShieldObject.direction=direction;
           end
        end
        function actuatorWrite(MotoShieldObject, percentValue) %-actuate in percent
            if (percentValue < MotoShieldObject.minDuty) && (percentValue ~= 0) %-if input value is 0, motor turns off
                percentValue = MotoShieldObject.minDuty; %-if input value is lesser than minimal Duty, make it minimal
            end
            switch MotoShieldObject.SHIELDRELEASE
                case 'R1'
                    writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN, (percentValue / 100));
                case 'R2'
                    if (MotoShieldObject.direction)
                        writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN1, (percentValue / 100));
                        writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN2, 0);
                    else
                        writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN1, 0);
                        writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN2, (percentValue / 100));
                    end
            end
        end
        function actuatorWriteVolt(MotoShieldObject, voltValue) %-actuate in percent
            switch MotoShieldObject.SHIELDRELEASE
                case 'R1'
                    writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN, (voltValue^2/5^2));
                case 'R2'
                    if (MotoShieldObject.direction)
                        writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN1, (voltValue^2/5^2));
                        writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN2, 0);
                    else
                        writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN1, 0);
                        writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN2, (voltValue^2/5^2));
                    end
            end
        end
        function ref = referenceRead(MotoShieldObject) %-returns reference of potentiometer in percent
           ref = readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_RPIN) * 100 / MotoShieldObject.VREF;
        end
        function voltageDrop = sensorReadVoltage(MotoShieldObject) %-voltageDrop across the shunt resistor in V
            switch MotoShieldObject.SHIELDRELEASE
                case 'R1'
                voltageDrop = readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_YV1)-readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_YV2); 
                case 'R2'
                voltageDrop = 0;
            end
        end
        function voltageDropAmp1 = sensorReadVoltageAmp1(MotoShieldObject) %-output from differential OpAmp in V
          switch MotoShieldObject.SHIELDRELEASE
            case 'R1'
                voltageDropAmp1 = readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_YAMP1)*100/5;
            case 'R2'
                voltageDropAmp1 = 0;
          end
        end
        function voltageDropAmp2 = sensorReadVoltageAmp2(MotoShieldObject) %-output from non-inverting OpAmp in V (not amplified value)
            switch MotoShieldObject.SHIELDRELEASE
                case 'R1'
                voltageDropAmp2 = readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_YAMP2)/MotoShieldObject.AMP_GAIN * 100 / 5;
                case 'R2'
                    voltageDropAmp2 = 0;
            end
        end
        function current = sensorReadCurrent(MotoShieldObject) %-Ohms law # current calculation
            switch MotoShieldObject.SHIELDRELEASE
                case 'R1'
                current = voltageDropAmp2 / MotoShieldObject.SHUNT * 1000.0; % return in mA
                case 'R2'
                current = readVoltage(MotoShieldObject.arduino,MotoShieldObject.MOTO_YPIN) / 100; % return current in A
            end
        end
        function calibration(MotoShieldObject) %-calibration
        writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN, 1);   %-actuate
        pause(1);   %-wait for motor to spin at maxRPM
        MotoShieldObject.maxRPM = readSpeed(MotoShieldObject.encoder);
        writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN, 0);%-stop the motor
        pause(0.5); %-wait for motor to stop spinning
        i = 0.13; %-initial duty in %, presumed potentionally minimal
        resetCount(MotoShieldObject.encoder); %-resets count to 0
        while(1) %-infinite loop
            writePWMDutyCycle(MotoShieldObject.arduino, MotoShieldObject.MOTO_UPIN, i);%-actuate
            pause(0.3); %-wait for at least one revolution           
            if abs(readCount(MotoShieldObject.encoder)) > 8 %-detecting motion
                pause(1); %-%-wait for motor to spin at minRPM
                MotoShieldObject.minDuty = i*100; %-if motor is spinning, this is minimal duty
                MotoShieldObject.minRPM = readSpeed(MotoShieldObject.encoder); %-sense minimal RPM
                actuatorWrite(MotoShieldObject,0); %-turn off motor
                break; %-exit the loop
            end
            i=i+0.01; %-increment
            if i>1
                error('[Calibration failed]: not able to detect minDuty')
                actuatorWrite(MotoShieldObject,0); %-turn off motor
                while(1)
                end
            end
        end %-end of while loop
        end %- end of function
        
        function rpm = sensorReadRPM(MotoShieldObject) %-sensing RPM # proper nomenclature
        rpm = readSpeed(MotoShieldObject.encoder);
        end
        function rpmPerc = sensorRead(MotoShieldObject) %-sensing RPM in percents # constrained
        rpmPerc = constrain(map(readSpeed(MotoShieldObject.encoder),MotoShieldObject.minRPM,MotoShieldObject.maxRPM,0,100),0,100);
        end
        function rpm = sensorReadRadian(MotoShieldObject) 
        rpm = readSpeed(MotoShieldObject.encoder)/60*2*pi;
        end
    end
end
