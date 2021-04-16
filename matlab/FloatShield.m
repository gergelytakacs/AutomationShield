%   FloatShield MATLAB API.
%
%   This script contains FloatShield class definition including
%   functions for FloatShield initialisation, calibration, 
%   acquisition of measured distances and of potentiometer runner
%   position. Purpose of this class is to make the FloatShield
%   operation in MATLAB more effective.   
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Peter Chmurciak.
%   Last update: 10.7.2019.

classdef FloatShield < handle             % FloatShield class definition

    properties(Access = public)           % Public class objects
        arduino;                          % Object for arduino board
        laserSensor;                      % Object for I2C laser sensor
    end

    properties(Access = private)          % Private class variables
        FLOAT_UPIN = 'D3';                % Fan (Actuator) pin
        FLOAT_RPIN = 'A0';                % Potentiometer (Reference) pin
        minDistance = 17;                 % Minimal distance measured by sensor - approximate value
        maxDistance = 341;                % Maximal distance measured by sensor - approximate value
    end

    methods(Access = public)                                                            % Public class functions

        function begin(FloatShieldObject, aPort, aBoard)                                % Initialisation function
            listOfLibraries = listArduinoLibraries();                                   % List of available arduino libraries
            libraryIsPresent = sum(ismember(listOfLibraries,'Pololu/Pololu_VL53L0X'));  % Check if required library is present
            if ~libraryIsPresent                                                        % If not, throw an error
                error('Pololu/Pololu_VL53L0X library was not detected by MATLAB! Use listArduinoLibraries() command to verify its presence.')                
            end
            FloatShieldObject.arduino = arduino(aPort, aBoard, 'Libraries', 'Pololu/Pololu_VL53L0X')  % Initialising object for arduino board
            FloatShieldObject.laserSensor = addon(FloatShieldObject.arduino, 'Pololu/Pololu_VL53L0X') % Initialising object for laser sensor
            beginSensor(FloatShieldObject.laserSensor)                                                % Initialising laser sensor
            disp('FloatShield initialized.')
        end

        function calibrate(FloatShieldObject)                          % Calibration function
            actuatorWrite(FloatShieldObject, 100)                      % Set fan speed to maximum
            while sensorReadDistance(FloatShieldObject) > 100          % Wait until the ball is in the upper third of the tube
                pause(0.1)
            end
            pause(1)                                     % Wait for ball to stabilise
            sum = 0;                                     % Variable for summing up the readings
            for i = 1:100                                % Sum one hundred sensor readings with sampling period 5 milliseconds
                sum = sum + sensorReadDistance(FloatShieldObject);
                pause(0.025)
            end
            FloatShieldObject.minDistance = sum / 100;                 % Set minDistance variable equal to average reading

            actuatorWrite(FloatShieldObject, 0)                        % Set fan speed to zero
            while sensorReadDistance(FloatShieldObject) < 300          % Wait until the ball is in the lower third of the tube
                pause(0.1)
            end
            pause(1)                                     % Wait for ball to stabilise
            sum = 0;                                     % Clear variable for summing up the readings
            for i = 1:100                                % Sum one hundred sensor readings with sampling period 5 milliseconds
                sum = sum + sensorReadDistance(FloatShieldObject);
                pause(0.025)
            end
            FloatShieldObject.maxDistance = sum / 100;         % Set maxDistance variable equal to average reading
            disp('FloatShield calibrated.')
        end

        function actuatorWrite(FloatShieldObject, percent)                                                    % Actuator write function - for controlling fan speed
            coercedInput = constrain(percent, 0, 100);                                                        % Coerce the iput value
            writePWMDutyCycle(FloatShieldObject.arduino, FloatShieldObject.FLOAT_UPIN, (coercedInput / 100)); % Set the PWM duty cycle based on the user input
        end

        function reference = referenceRead(FloatShieldObject)                                            % Reference read function 
            reference = readVoltage(FloatShieldObject.arduino, FloatShieldObject.FLOAT_RPIN) * 100 / 5;  % Read the percentual position of potentiometer runner
        end

        function minimalDistance = returnMinDistance(FloatShieldObject)  % Return minimal distance function
            minimalDistance = FloatShieldObject.minDistance;             % Returns minimal distance in millimetres measured by sensor according to calibration
        end

        function maximalDistance = returnMaxDistance(FloatShieldObject)  % Return maximal distance function
            maximalDistance = FloatShieldObject.maxDistance;             % Returns maximal distance in millimetres measured by sensor according to calibration
        end

        function ballPositionPercent = sensorRead(FloatShieldObject)            % Sensor read function - returns percentual altitude of the ball
            distance = FloatShieldObject.sensorReadDistance();                  % Distance in millimetres reading
            mapped = map(distance, FloatShieldObject.maxDistance, FloatShieldObject.minDistance, 0, 100); % Transforms the distance to percentual altitude              
            ballPositionPercent = constrain(mapped, 0, 100);                    % Coerces the percentual altitude
        end

        function ballAltitude = sensorReadAltitude(FloatShieldObject)      % Sensor read altitude function - returns altitude of the ball in millimetres
            reading = sensorReadDistance(FloatShieldObject);               % Reads the distance in millimetres between the sensor and the ball
            ballAltitude = FloatShieldObject.maxDistance - reading;        % Transforms the distance to altitude of the ball
        end

        function rawSensorReading = sensorReadDistance(FloatShieldObject)  % Sensor read distance function - returns distance between the sensor and the ball
            rawSensorReading = readSensor(FloatShieldObject.laserSensor);  % Measure the distance in millimetres with laser sensor
        end

    end  
    
end                                                                        % End of class definition                                         