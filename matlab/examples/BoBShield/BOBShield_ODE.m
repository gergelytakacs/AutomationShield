function [dx, y] = BOBShieldSin_ODE(t, x, u, H, varargin)
%   ODE file representing the dynamics of the FloattShield device.
%   Reference: Chaos, D. et al - Robust switched control of an air
%   levitation system with minimum sensing, ISA Transactions, 2019
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Martin Gulan and Gergely Takács.

% States: x1 - ball position, x2 - ball speed, x3 - airspeed inside the tube
% Input:  u - fan voltage
% Output: y - ball position
%
%2nd order linear
y = x(1);                                               % Output equation
dx(1) = x(2);                                           % State equation...
dx(2) =H*u; % ...
