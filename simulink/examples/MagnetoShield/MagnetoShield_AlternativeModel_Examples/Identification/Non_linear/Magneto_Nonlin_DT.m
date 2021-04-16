%   DT non-linear alternative model of the MagnetoShield
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

function [dx] = Magneto_Nonlin_DT(x,u,T)
g = 9.81;
m = 0.74e-3;
L = 0.39071;
R = 213.5237;
Km = 1.8752e-08;
Ke = 4.4449e-07;
C = 0;

dx1 = T*x(2)+x(1);
dx2 = T*g-T*Km*x(3)/(m*x(1)^4)+(1-T*C)*x(2);
dx3 = (1-T*R/L)*x(3)+T*Ke*x(2)/(L*x(1)^4)+T/L*u(1);

dx = [dx1; dx2; dx3];