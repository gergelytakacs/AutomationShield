%   AutomationShield map function for rescaling values
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Peter Chmurciak. 
%   Last update: 9.7.2019.

function remappedValue = map(originalValue, fromLow, fromHigh, toLow, toHigh)            
        remappedValue = (originalValue - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
end

