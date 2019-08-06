%   Generates a PRBS sequence and saves as a C header for Arduino
%
%   This function takes three arguments: a string naming
%   the variable and the resulting header file containing the pseudo
%   random binary (PRBS) sequence, the number of samples the sequence
%   shall last, minimal and maximal levels and the bandwidth. 
%   the bandwidth is scaled to unit sampling time, therefore 1 is
%   your sampling, while 1/10 is 10 times slower. The result is displayed
%   in a plot for illustration.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by:      Gergely Takács.
%   Last updated by: Gergely Takács
%   Last update on:  6.8.2019.

function prbsGenerate(varName,N,minu,maxu,B);
band = [0, B];                % Frequency content
levels = [minu maxu];         % Signal levels
prbs = idinput(N,'prbs',band,levels); 

plot(prbs)                    % Plot signal
xlabel('Samples (-)')
ylabel('Amplitude (-)')
title('PRBS Signal')
vectorToC(prbs,varName,'int'); % Store to C
