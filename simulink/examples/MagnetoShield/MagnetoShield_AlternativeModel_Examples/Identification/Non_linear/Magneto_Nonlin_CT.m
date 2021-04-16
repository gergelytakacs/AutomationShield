%   CT non-linear alternative model of the MagnetoShield
%
%   Non-linear model is based on Jakub Mihalik's thesis work which can be
%   found here:
%   https://github.com/gergelytakacs/AutomationShield/wiki/Publications
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%
%   Created by:       Jakub Mihalik
%   Created on:       15.4.2021
%   Last updated by:  Jakub Mihalik
%   Last update on:   15.4.2021

function [dx] = Magneto_Nonlin_CT(t,x,u)
g = 9.81;
m = 0.74e-3;
L = 0.39071;
R = 213.5237;
Km = 1.8752e-08;
Ke = 4.4449e-07;
C = 0;

dx1 = x(2);
dx2 = g-Km/m*x(3)/x(1)^4-C*x(2); 
dx3 = -R/L*x(3)+Ke/(L*x(1)^4)*x(2)+1/L*u(1);

dx = [dx1;dx2;dx3];