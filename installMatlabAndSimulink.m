%   Installs AutomationShield MATLAB and Simulink API.
%  
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 22.10.2018.

function installForMATLAB()
    thisdir=pwd;
    addpath(genpath(thisdir));
    installForMATLAB();
    installForSimulink();
end