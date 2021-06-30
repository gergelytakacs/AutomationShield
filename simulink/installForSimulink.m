%   Installs AutomationShield Simulink API.
%  
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 30.6.2021.

function installAutomationShield()
    if(verLessThan('Simulink','9.4'))                           % Checking for Simulink version
        error('Please install MATLAB/Simulink 2019a or newer! The AutomationShield Simulink API is not supported by your MATLAB/Simulink installation.') % Simulink installation incompatible
    end
    thisdir=pwd;
    addpath(genpath(thisdir));
    savepath
    disp('AutomationShield Simulink API added to MATLAB path.')

end