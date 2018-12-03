function [dx, y] = MagnetoShield_ODE_3(t, x, u, K, R, L, varargin)
%   ODE file representing the dynamics of the MagnetoShield device.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
% See https://pdfs.semanticscholar.org/398d/a5c7d5e0fb00ea4d12175daf0c3792b18247.pdf

g  =  9.80665; % [m/s^2]
m  =  0.76E-3;
y  = [x(1)];  % Output [m]
dx = [x(2);   % Nonlinear state-space dynamics
     g-(K/m)*(x(3)^2/x(1)^2);
     -(R/L)*x(3)+(2*K/L)*x(2)*x(3)/x(1)+(1/L)*u(1)]; 
       
       