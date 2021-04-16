%   MAGNETOSHIELD Linearization "Alternative"
%
%   This example shows a symbolic linearization preocess of the non-linear 
%   "alternative" model described by differential equations [dx1 dx2 dx3]'.
%   It results in LTI statespace model represented by symbolic matrices A 
%   and B.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   If you have found any use of this code, please cite our work in your
%   academic publications, such as theses, conference articles or journal
%   papers. A list of publications connected to the AutomationShield
%   project is available at: 
%   https://github.com/gergelytakacs/AutomationShield/wiki/Publications
%
%   Created by:       Jakub Mihalik
%   Created on:       5.12.2020
%   Last updated by:  Jakub Mihalik
%   Last update on:   15.4.2021

%% Linearization
syms x1 x2 x3 x01 x02 x03 R L Km Ke u g m C s kon

% System state variables [position; velocity; current]
x = [x1; x2; x3];

% Non-linear statespace system structure
dx1 = x2;
dx2 = g-Km/m*(x3)/x1^4-C*x2;
dx3 = -R/L*x3+Ke*1/(L*x1^4)*x2+1/L*u;

% Jacobian matrix of first derivatives
J = jacobian([dx1 dx2 dx3],[x1 x2 x3]);

% Matrix A of the linear system
A = subs(J,[x1 x2 x3],[x01 x02 x03]);
% Matrix B of the linear system
B = [diff(dx1,u);diff(dx2,u);diff(dx3,u)];
C = [1 0 0; 0 0 1];
