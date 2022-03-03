

classdef skuska < matlabshared.addon.LibraryBase
    properties(Access = private, Constant = true)
        VL6180X_CREATE  =  hex2dec('00')
        VL6180X_BEGIN   =  hex2dec('01')
        VL6180X_READ    =  hex2dec('02')
        VL6180X_DELETE  =  hex2dec('03')
        READ_COMMAND = hex2dec('04')
    end 
    properties(Access = protected, Constant = true)
        LibraryName = 'skuska/skuska'
        DependentLibraries = {}
        LibraryHeaderFiles = {}
        CppHeaderFile = fullfile(arduinoio.FilePath(mfilename('fullpath')), 'src', 'skuska.h')
        CppClassName = 'skuska'
    end
    methods
        % Constructor
        function obj = skuska(parentObj)
            obj.Parent = parentObj;
            create_VL6180X(obj);
        end
        
        % pomocna na overenie funkcnosti
        function out = read(obj)
            cmdID = obj.READ_COMMAND;
            inputs = [];
            output = sendCommand(obj, obj.LibraryName, cmdID, inputs);
            out = char(output');
        end

        function beginSensor(obj)            
            cmdID = obj.VL6180X_BEGIN;
            success = sendCommand(obj, obj.LibraryName, cmdID, []);
            if ~success            
                error('Error initialising VL6180X sensor!');
            end            
        end

        % Function for sensor object creation
        function create_VL6180X(obj)
            try                
                cmdID = obj.VL6180X_CREATE;                
               % configurePinResource(obj.Parent, 'A4', obj.ResourceOwner, 'I2C');
               % configurePinResource(obj.Parent, 'A5', obj.ResourceOwner, 'I2C');                
                sendCommand(obj, obj.LibraryName, cmdID, []);                
            catch e
                throwAsCaller(e);
            end
        end

        % Function for reading data from sensor
        function out = readSensor(obj)
            cmdID = obj.VL6180X_READ;
            out = sendCommand(obj, obj.LibraryName, cmdID, []);
            
        end
    end
end