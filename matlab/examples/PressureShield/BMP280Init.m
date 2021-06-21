%   MATLAB function to initialize temperature and pressure
%   sensor BMP280
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Martin Staron.
%   Last update: 26.4.2021.

function y = BMP280Init(dev),
             writeRegister(dev,'F4',0b1111);
             writeRegister(dev,'F5',0b1000);
end