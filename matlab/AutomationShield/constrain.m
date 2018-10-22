%   AutomationShield general saturation
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 22.10.2018.

function u = constrain(u,umin,umax)
if u>=umax
    u=umax;
elseif u<=umin;
    u=umin;
end
end