function [dx, y] = FloatShield_ODE(t, x, u, m, cd, ro, A, g, k, tau, varargin)
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
y = x(1);                                               % Output equation
dx(1) = x(2);                                           % State equation...
dx(2) = (1/(2*m))*cd*ro*A*(x(3)-x(2))*abs(x(3)-x(2))-g; % ...dynamics of the floating ball
dx(3) = (k*u-x(3))/tau;                                 % ...airspeed - fan input relation
