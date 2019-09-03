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

switch nargin                                % Handling argument numbers
        case 2                               % If binary levels not defined,
            minu=-1;                         % use -1,+1
            maxu=1;                          % and
            B=1;                             % assume unit band.
        case 4                               % If band not defined
            B=1;                             % assume unit band.
        case 5
                                             % default case.
        otherwise
            error('Check your arguments: variable name, length, upper limit, lower limit and unit band portion.')
end
    
band = [0, B];                               % Frequency content
levels = [minu maxu];                        % Signal levels
warning('off', 'Ident:dataprocess:idinput7') % Do not warn that only a portion is used
prbs = idinput(N,'prbs',band,levels);        % Generate PRBS

stairs(prbs)                                 % Plot signal
axis([0,N,1.1*minu,1.1*maxu])
xlabel('Samples (-)')                        % X-label                   
ylabel('Amplitude (-)')                      % Y-label
title('PRBS Signal')                         % Title

vectorToC(prbs,varName,'int');               % Store to C
