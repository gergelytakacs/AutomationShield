%   Generates a PRBS sequence for the FloatShield and saves it as a
%   C header for Arduino
%
%   This function generates a pseudo-random binary sequence to
%   test the FloatShield
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by:      Gergely Takács.
%   Last updated by: Peter Chmurciak
%   Last update on:  29.8.2019.

N = 7200;                % Length (samples)
minu = -1;               % Minimum input
maxu = 1;                % Maximum input
B = 1 / 10;              % Upper passband (Unit sample / slow-down)

prbsGenerate('prbsU', N, minu, maxu, B) % Generate header "prbsU.h".