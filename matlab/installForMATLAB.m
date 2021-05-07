%   Installs AutomationShield MATLAB API.
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
savepath
disp('AutomationShield MATLAB API added to MATLAB path.')
installMPT3; % Comment this line if you don't want to use the explicit MPC examples
end
end