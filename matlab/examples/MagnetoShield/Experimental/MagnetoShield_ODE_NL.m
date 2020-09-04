%   MagnetoShield non-linear state-space model
% 
%   This file can be used for system identification and simulation
%   purposes and describes the behaviour of the MagnetoShield system
%   using a set of three first order differential equations derived
%   from a mathematical-physical analysis.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Jakub Mihalík and Gergely Takács. 
%   Last update: 25.09.2019.

function [dx,y] = MagnetoShield_ODE(t,x,u,K,m,R,L,g,varargin)

y = [x(1); x(3)];									% Outputs available: position, current
dx(1) = x(2);										% Order 2 -> 2 x Order 1
dx(2) =  g - K/m*x(3)^2/x(1)^2;         % Mechanical equation
dx(3) = -R/L*x(3)+1/L*u(1)+(2*K/L)*(x(3)*x(1)^2)*x(2);	    % Electrical equation
