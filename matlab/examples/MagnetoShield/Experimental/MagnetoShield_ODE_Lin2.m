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

function [A,B,C,D,K] = MagnetoShield_ODE_Lin2(par,ts,aux)

                                 % Parameter guess

A = [ 0       1     0 ;
      par(1)  0    -par(2);
      0    -par(3) -par(4)];

B = [ 0; 
      0; 
      par(5)];
C = [ 1 0 0;
      0 0 1 ];
D = zeros(2,1);
K = zeros(3,2);
X0 = [0 0 0];
% 
% if ts>0 % Sample the model with sample time Ts
%    s = expm([[A B]*ts; zeros(1,3)]);
%    A = s(1:2,1:2);
%    B = s(1:2,3);
% end