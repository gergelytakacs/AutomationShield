%   Generates an APRBS sequence and saves as a C header for Arduino
%
%   This function takes six arguments: a string naming
%   the variable and the resulting header file containing the amplitude
%   modulated pseudo random binary (APRBS) sequence, the number of samples
%   the sequence shall last, seed value used for generation of random
%   amplitude values, minimal and maximal levels and the bandwidth.
%   The bandwidth is scaled to unit sampling time, therefore 1 is
%   your sampling, while 1/10 is 10 times slower. The result is displayed
%   in a plot for illustration.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by:      Gergely Takács and Peter Chmurciak.
%   Last updated by: Peter Chmurciak
%   Last update on:  29.8.2019.

function aprbsGenerate(varName, N, randomSeed, minu, maxu, B)

switch nargin                              % Handling argument numbers
    case 2                                 % If seed value not defined,
        randomSeed = 1;                    % use 1 as a seed value
        minu = -1;                         % use -1,+1 binary levels
        maxu = 1;                          % and
        B = 1;                             % assume unit band.
    case 3                                 % If binary levels not defined,
        minu = -1;                         % use -1,+1
        maxu = 1;                          % and
        B = 1;                             % assume unit band.
    case 5                                 % If band not defined
        B = 1;                             % assume unit band.
    case 6
                                           % Default case.
    otherwise
        error('Check your arguments: variable name, length, seed value, upper limit, lower limit and unit band portion.')
end

band = [0, B];                                % Frequency content
levels = [minu, maxu];                        % Signal levels
warning('off', 'Ident:dataprocess:idinput7')  % Do not warn that only a portion is used
prbs = idinput(N, 'prbs', band, levels);      % Generate PRBS

seed = randomSeed;                            % Initialise seed value
range = abs(maxu-minu);                       % Calculate level range
for i = 1:length(prbs) - 1
    if prbs(i) == prbs(i+1)                   % If being on the same level
        rng(seed);                            % use same seed
        r = range * rand;                     % Random number from 0 to range
        if prbs(i) == maxu                    % If level is high
            aprbs(i) = prbs(i) - r;           % decrement
            aprbs(i+1) = prbs(i+1) - r;
        else                                  % If level is low
            aprbs(i) = prbs(i) + r;           % increment
            aprbs(i+1) = prbs(i+1) + r;
        end
    else
        if seed > 2^31                        % Prevent seed overflow
            seed = randomSeed;        
        end
        seed = seed + 10;                     % When level is changed,
        rng(seed);                            % increment seed value
        r = range * rand;
        if prbs(i+1) == maxu                  % If level is high
            aprbs(i+1) = prbs(i+1) - r;       % decrement
        else                                  % If level is low
            aprbs(i+1) = prbs(i+1) + r;       % increment
        end
    end
end

stairs(aprbs)                                 % Plot signal
axis([0, N, minu - 0.1 * range, maxu + 0.1 * range])
xlabel('Samples (-)')                         % X-label
ylabel('Amplitude (-)')                       % Y-label
title('APRBS Signal')                         % Title

vectorToC(aprbs, varName, 'float');           % Store to C
end