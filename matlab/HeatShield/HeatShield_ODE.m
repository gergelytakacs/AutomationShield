function [dx, y] = printerheat(t, x, u, alpha, beta, gamma, delta, varargin)
%   ODE file representing the dynamics of the HeatShield device.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 


y = [x(1)]; % Output
dx = [-(alpha/beta)*x(1)+(gamma/beta)*u(1)+(alpha/beta)*delta]; % Heat transfer eq
       
       