function [dx, y] = MagnetoShield_ODE_1(t, x, u, K, R, L, varargin)
%   ODE file representing the dynamics of the MagnetoShield device.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 

g  =  9.80665; % [m/s^2]
m  =  0.76E-3; % [kg] Magnet mass
y  = [x(1)];  % Output [m]
dx = [x(2);   % Nonlinear state-space dynamics
     g-(K/m)*(x(3)^2/x(1)^2);
     -(R/L)*x(3)+(1/L)*u(1)]; 
       
       