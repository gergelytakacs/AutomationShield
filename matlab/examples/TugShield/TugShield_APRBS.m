startScript;
%   Generates an APRBS sequence for the TugShield and saves it as a
%   C header for Arduino
%
%   This function generates a amplitude modulated pseudo-random binary
%   sequence to test the TugShield
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by:      Gergely Takács and Peter Chmurciak.
%   Last updated by: Eva Vargová.
%   Last update on:  8.2.2021.

N = 4800;                 % Length (samples)
seed = 100;               % Seed value for generating pseudorandom values
minu = 80;                % Minimum input
maxu = 150;               % Maximum input
B = 1 / 100;               % Upper passband (Unit sample / slow-down)

aprbsGenerate('aprbsU', N, seed, minu, maxu, B) % Generate header "aprbsU.h"
