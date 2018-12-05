function [dx, y] = MagnetoShield_ODE_2(t, x, u, alpha, beta, gamma, delta, varargin)
%   ODE file representing the dynamics of the MagnetoShield device.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 

g  = 9.80665; % [m/s^2]
y  = [x(1)];  % Output [m]
dx = [x(2);                                     ...  % Nonlinear state-space dynamics
     g-alpha*(x(3)^2/x(1)^2)-delta*(1/x(1)^2);  ...
     -beta*gamma*x(3)+gamma*u(1)                ...
     ]; 
       
       