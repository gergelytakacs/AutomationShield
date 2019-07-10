%   Matlab implementation of Pololu arduino library for 
%   VL53L0X Time-of-Flight distance sensor.
%  
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Peter Chmurciak. 
%   Last update: 9.7.2019.

% Sensor class definition, inheriting LibraryBase class properties
classdef Pololu_VL53L0X < arduinoio.LibraryBase          

    % Define command IDs for commandHandler method
    properties(Access = private, Constant = true)
        Pololu_VL53L0X_CREATE  =  hex2dec('00')
        Pololu_VL53L0X_BEGIN   =  hex2dec('01')
        Pololu_VL53L0X_READ    =  hex2dec('02')
        Pololu_VL53L0X_DELETE  =  hex2dec('03')
    end

    % Define the required names and pathways to used source files
    properties(Access = protected, Constant = true)
        LibraryName = 'Pololu/Pololu_VL53L0X'
        DependentLibraries = {}
        ArduinoLibraryHeaderFiles = {}
        CppHeaderFile = fullfile(arduinoio.FilePath(mfilename('fullpath')), 'src', 'Pololu_VL53L0X.h')
        CppClassName = 'Pololu_VL53L0X'
        ResourceOwner = 'Pololu/Pololu_VL53L0X'
    end
    
    % Hidden methods (not meant to be used directly by the user) with unrestricted access
    methods(Hidden, Access = public)
        
        % Constructor
        function obj = Pololu_VL53L0X(parentObj)
            obj.Parent = parentObj;
            obj.Pins = {'A4', 'A5'};
            count = getResourceCount(obj.Parent, obj.ResourceOwner);            
            if count > 0
                error('You can use only one VL53L0X sensor!');
            end
            incrementResourceCount(obj.Parent, obj.ResourceOwner);
            createPololu_VL53L0X(obj);
        end
        
        % Function for sensor object creation
        function createPololu_VL53L0X(obj)
            try                
                cmdID = obj.Pololu_VL53L0X_CREATE;                
                configurePinResource(obj.Parent, 'A4', obj.ResourceOwner, 'I2C');
                configurePinResource(obj.Parent, 'A5', obj.ResourceOwner, 'I2C');                
                sendCommand(obj, obj.LibraryName, cmdID, []);                
            catch e
                throwAsCaller(e);
            end
        end
    end
    
    % Protected methods (not meant to be used directly by the user)
    methods(Access = protected)
        
        % Function for sensor object removal
        function delete(obj)
            try
                parentObj = obj.Parent;                
                for iLoop = obj.Pins
                    configurePinResource(parentObj, iLoop{:}, obj.ResourceOwner, 'Unset');
                end                
                decrementResourceCount(parentObj, obj.ResourceOwner);
                cmdID = obj.Pololu_VL53L0X_DELETE;
                sendCommand(obj, obj.LibraryName, cmdID, []);
            catch               
            end
        end
    end
    
    % Public methods - meant to be used directly by the user
    methods(Access = public)
        
        % Function for sensor initialisation
        function beginSensor(obj)            
            cmdID = obj.Pololu_VL53L0X_BEGIN;
            success = sendCommand(obj, obj.LibraryName, cmdID, []);
            if ~success            
                error('Error initialising VL53L0X sensor!');
            end            
        end

        % Function for reading data from sensor
        function out = readSensor(obj)
            cmdID = obj.Pololu_VL53L0X_READ;
            out = sendCommand(obj, obj.LibraryName, cmdID, []);
            out = 256 * out(1) + out(2);
        end
        
        % Function for sensor object removal
        function removeSensor(obj)
            delete(obj);            
        end
        
    end

end