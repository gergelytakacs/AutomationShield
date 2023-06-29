%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%   Created by Jan Boldocky.
%   Last update: 16.1.2023.
clear;clc;close all

load AeroShield_GreyboxModel_Linear.mat
Ts = 0.003; % Sample time
dlinsys = c2d(linsys,Ts); %Discrete model
Q = [100 0;
     0 1]; % State penalty
R = 1;   % Actuator penalty
K = lqr(dlinsys,Q,R) % Optimal gain without integrator
eig(dlinsys.A-dlinsys.B*K)

%% with integrator
intR = 3e3
intQ = diag([1 1 100]);
intA = [ones(1,1),-dlinsys.C(1,:);
     zeros(2,1),dlinsys.A];
intB = [0; dlinsys.B;];
intC = [0 dlinsys.C(1,:)];
intdsys = ss(intA,intB,intC,0,Ts)
intK = lqr(intdsys,intQ,intR)

eig(intA-intB*intK)