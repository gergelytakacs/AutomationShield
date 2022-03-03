%   Matlab implementation of VL6180x arduino library for 
%   VL6180X Time-of-Flight distance sensor.
%  
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Michal Bíro. 
%   Last update: 22.2.2022.

% Sensor class definition, inheriting LibraryBase class properties
classdef VL6180X < matlabshared.addon.LibraryBase          

    % Define command IDs for commandHandler method
    properties(Access = private, Constant = true)
        VL6180X_CREATE  =  hex2dec('00')
        VL6180X_BEGIN   =  hex2dec('01')
        VL6180X_READ    =  hex2dec('02')
        VL6180X_DELETE  =  hex2dec('03')
    end

    % Define the required names and pathways to used source files
    properties(Access = protected, Constant = true)
        LibraryName = 'VL6180X/VL6180X'
        DependentLibraries = {}
        LibraryHeaderFiles = {}        
        CppHeaderFile = fullfile(arduinoio.FilePath(mfilename('fullpath')), 'src', 'VL6180X.h')
        CppClassName = 'VL6180X'
        %%ResourceOwner = 'ML/ML_VL6180X'
        Pins = {'A4', 'A5'}
    end
    
    % Hidden methods (not meant to be used directly by the user) with unrestricted access
    methods(Hidden, Access = public)
        
        % Constructor
        function obj = VL6180X(parentObj)
            obj.Parent = parentObj;            
            count = getResourceCount(obj.Parent, obj.ResourceOwner);            
            if count > 0
                error('You can use only one VL6180X sensor!');
            end
            incrementResourceCount(obj.Parent, obj.ResourceOwner);
            createVL6180X(obj);
        end
        
        % Function for sensor object creation
        function createVL6180X(obj)
            try                
                cmdID = obj.VL6180X_CREATE;                
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
                cmdID = obj.VL6180X_DELETE;
                sendCommand(obj, obj.LibraryName, cmdID, []);
            catch               
            end
        end
    end
    
    % Public methods - meant to be used directly by the user
    methods(Access = public)
        
        % Function for sensor initialisation
        function beginSensor(obj)            
            cmdID = obj.VL6180X_BEGIN;
            success = sendCommand(obj, obj.LibraryName, cmdID, []);
            if ~success            
                error('Error initialising VL6180X sensor!');
            end            
        end

        % Function for reading data from sensor
        function out = readSensor(obj)
            cmdID = obj.VL6180X_READ;
            inputs=[]
            output = sendCommand(obj, obj.LibraryName, cmdID, inputs);
            out = 256 * output(1) + output(2);
        end
        
        % Function for sensor object removal
        function removeSensor(obj)
            delete(obj);            
        end
        
    end

end