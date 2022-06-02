%   MotoShield Linear Quadratic Controller Gain Calculation.
% 
%   
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Ján Boldocký.

startScript;

load identification_data.mat                             % Read identification results
load MotoShield_LinearSS.mat 
%%
Ts=0.02
zmodel = c2d(model,Ts);
intA = [zmodel.A, zeros(2, 1); -zmodel.C(1,:), ones(1,1)];
intB = [zmodel.B; 0];
intC = [zmodel.C(1,:), 0];
dsys=ss(intA,intB,intC,0,Ts)
Q_lqr=diag([1 1 1]);
R_lqr=1e8;
K=lqrd(intA,intB,Q_lqr,R_lqr,Ts)
eig(intA-intB*K);

