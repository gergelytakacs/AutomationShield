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
%   Created by Gergely Takacs. 
%   Last update: 25.09.2019.

function [A,B,C,D,K] = MagnetoShield_ODE(par,ts,aux)

m  =  par(1);
Km =  par(2);
R  =  par(3);
L  =  par(4);
Ke =  par(5);

y0=   aux(1);
u0=   aux(2);
i0=   aux(3);

alpha   = 2*(Km/m)*u0^2/y0^3;                   % Linearized parameter guess
beta    = 2*(Km/m)*u0/y0^2;                     % Linearized parameter guess
gamma   = 2*Ke*i0/(L*y0^2);                     % Linearized parameter guess
delta   = R/L;                                  % Linearized parameter guess
epsilon = 1/L;                                  % Linearized parameter guess

A = [ 0      1      0 ;
      alpha  0     -beta;
      0     -gamma -delta];

B = [ 0; 
      0; 
      epsilon ];
C = [ 1 0 0;
      0 0 1 ];
D = zeros(2,1);
K = zeros(3,2);
X0 = [0; 0; 0];

% if ts>0 % Sample the model with sample time Ts
%    s = expm([[A B]*ts; zeros(1,3)]);
%    A = s(1:2,1:2);
%    B = s(1:2,3);
% end