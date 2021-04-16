%   MAGNETOSHIELD Linearization
%
%   This example shows a symbolic linearization preocess of the non-linear 
%   model described by differential equations [dx1 dx2 dx3]'. It results in
%   LTI statespace model represented by symbolic matrices A and B.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   If you have found any use of this code, please cite our work in your
%   academic publications, such as thesis, conference articles or journal
%   papers. A list of publications connected to the AutomationShield
%   project is available at: 
%   https://github.com/gergelytakacs/AutomationShield/wiki/Publications
%
%   Created by:       Jakub Mihalik
%   Created on:       5.12.2020
%   Last updated by:  Jakub Mihalik
%   Last update on:   22.12.2020

syms x1 x2 x3 x01 x02 x03 R L K u g m C s Ke Km

x = [x1; x2; x3]; % System state variables [position; velocity; current]
x0 = [x01; x02; x03]; % Linearization points

% Non-linear system equations
dx1 = x2;
dx2 = g-Km/m*x3^2/x1^2;
dx3 = -R/L*x3-2*Ke*x3/(L*x1^2)*x2+1/L*u;

% Jacobian matrix of partial derivatives of the system
J = jacobian([dx1 dx2 dx3],[x1 x2 x3]);

% Matrix A of the linear SS system
A = subs(J,x,x0)
% Matrix B of the linear SS system
B = [diff(dx1,u);diff(dx2,u);diff(dx3,u)]


