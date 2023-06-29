% Generating an APRBS sequence for the MotoShield identification.
% Generated C header file is located in the following directory: 
% /examples/MotoShield/MotoShield_Identification/aprbsU.h

%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
close all

N = 4800;                 % Length (samples)
seed = 10;               % Seed value for generating pseudorandom values
minu = 0.7;                % Minimum input
maxu = 2;                 % Maximum input
B = 1 / 185;               % Upper passband (Unit sample / slow-down)
aprbsGenerate('aprbsU', N, seed, minu, maxu, B) % Generate header "aprbsU.h"