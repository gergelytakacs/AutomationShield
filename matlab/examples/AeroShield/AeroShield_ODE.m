function [dx, y] = AeroShield_ODE(t, x, u, m, g, l, c, K, varargin)
%   ODE file representing the dynamics of the FloattShield device.
%   Reference: Chaos, D. et al - Robust switched control of an air
%   levitation system with minimum sensing, ISA Transactions, 2019
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Jan Boldocky.

% States: x1 - ball position, x2 - ball speed, x3 - airspeed inside the tube
% Input:  u - fan voltage
% Output: y - ball position
y = x(1);                                               % Output equation
dx(1) = x(2);                                           % State equation...
dx(2) = u*(K/(l*m))-(g/l)*sin(x(1))-x(2)*(c/(m*l^2));