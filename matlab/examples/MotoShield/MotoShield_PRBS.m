% Generating an PRBS sequence for the MotoShield identification.
% Generated C header file is located in the following directory: 
% /examples/MotoShield/MotoShield_Identification/prbsU.h

%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.

N = 7200;                % Length (samples)
minu = 3.8;               % Minimum input
maxu = 4.6;                % Maximum input
B = 1 / 70;              % Upper passband (Unit sample / slow-down)

prbsGenerate('prbsU', N, minu, maxu, B) % Generate header "prbsU.h".