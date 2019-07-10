%   AutomationShield PID class for computing PID regulator output.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Peter Chmurciak.
%   Last update: 10.7.2019.

classdef PID < handle                      % PID class definition
    properties(Access = private)           % Private class variables
        Ts = 0;                            % Sampling period in seconds
        Kp = 0;                            % Proportional Kp constant of PID regulator
        Ti = 0;                            % Integral Ti constant of PID regulator
        Td = 0;                            % Derivative Td constant of PID regulator
        eSum = 0;                          % Variable for storing sum of previous errors
        e = 0;                             % Variable for storing value of current error
        ep = 0;                            % Variable for storing value of previous error
        check = [0, 0, 0, 0];              % Check that all parameters have been set
    end

    methods(Access = public)               % Public class functions
        function setKp(PIDobject, aKp)     % setKp(Kp)
            PIDobject.Kp = aKp;            % Set Kp parameter of PID regulator
            PIDobject.check(1) = 1;        % Check that Kp has been set
        end

        function setTi(PIDobject, aTi)     % setTi(Ti)
            PIDobject.Ti = aTi;            % Set Ti parameter of PID regulator
            PIDobject.check(2) = 1;        % Check that Ti has been set
        end

        function setTd(PIDobject, aTd)     % setTd(Td)
            PIDobject.Td = aTd;            % Set Td parameter of PID regulator
            PIDobject.check(3) = 1;        % Check that Td has been set
        end

        function setTs(PIDobject, aTs)     % setTs(Ts)
            PIDobject.Ts = aTs;            % Set sampling period for PID regulator
            PIDobject.check(4) = 1;        % Check that Ts has been set
        end

        function setParameters(PIDobject, aKp, aTi, aTd, aTs) % setParameters(Kp,Ti,Td,Ts)
            PIDobject.Kp = aKp;                               % Set all parameters for PID regulator at once
            PIDobject.Ti = aTi;
            PIDobject.Td = aTd;
            PIDobject.Ts = aTs;
            PIDobject.check = [1, 1, 1, 1];                   % Check that all parameters have been set
        end

        function parameters = getParameters(PIDobject)                             % getParameters()
            parameters = [PIDobject.Kp, PIDobject.Ti, PIDobject.Td, PIDobject.Ts]; % Get all parameters of PID regulator in a form of row vector - [Kp,Ti,Td,Ts]
        end

        % compute(currentError, saturationMin, saturationMax, antiWindupMin, antiWindupMax)
        % Computes the output of PID regulator based on provided constants and values
        function outputU = compute(PIDobject, aError, aSaturationMin, aSaturationMax, aAntiWindupMin, aAntiWindupMax)
            if (sum(PIDobject.check) == 4)                                          % Check if all parameters have been set
                PIDobject.e = aError;
                PIDobject.eSum = PIDobject.eSum + PIDobject.e;
                PIDobject.eSum = constrain((PIDobject.Kp * PIDobject.Ts / PIDobject.Ti)*PIDobject.eSum, aAntiWindupMin, aAntiWindupMax) / (PIDobject.Kp * PIDobject.Ts / PIDobject.Ti);
                u = PIDobject.Kp * (PIDobject.e+((PIDobject.Ts / PIDobject.Ti) * PIDobject.eSum) + ((PIDobject.Td / PIDobject.Ts) * (PIDobject.e-PIDobject.ep)));
                PIDobject.ep = PIDobject.e;
                outputU = constrain(u, aSaturationMin, aSaturationMax);
            else
                error('Make sure that all parameters (Kp,Ti,Td,Ts) have been set.') % Throw error if all parameters have not been set
            end
        end       
    end   % End of public methods definiton
end   % End of class definition