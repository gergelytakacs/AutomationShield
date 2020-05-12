%   Installs AutomationShield Simulink API.
%  
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 17.10.2018.

function installAutomationShield()
    thisdir=pwd;
    addpath(genpath(thisdir));
    savepath
    disp('AutomationShield Simulink API added to MATLAB path.')
end