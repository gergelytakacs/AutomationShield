%   MAGNETOSHIELD Pole-placement simulation and control initialization for Simulink
%
%   Code below is used as an initialization script for MagnetoShield system
%   and default properties can be changed via this script.
%   This example reads the linearized model of the MagnetoShield device,
%   then expands it with an integrator and then computes Pole-placement
%   gain based on choosen poles. For computation the Ackermann's formula
%   was used. For estimation of the states is used Luenberger's observer
%   and suitable form of Ackermann's formula was applied as well.
%   Consequently are the model properties as stability, controlability,
%   observability, step and impulse response evaluated and displayed. 
%   Root-locus plot is shown too.
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
%   Last update on:   22.12.2020

clear;close all;clc;
load '../../../matlab/examples/MagnetoShield/MagnetoShield_Models_Greybox_SS';

%% MagnetoShield - Pole-placement simulation and control initialization for Simulink
Ts = 0.001;                   % Sampling of the Simulink simulation

% Matrices extraction from model
modeld=c2d(model,Ts);           % Discretized linear state-space model
A = modeld.a;                   % Extract A
B = modeld.b;                   % Extract B
C = [1 0 0];                   % Position is measured for integrator 'xi' computation

% Discrete augmented model with integrator state - unstable, unobservable
Ai = [1 -C;
      zeros(3,1) A];
Bi = [0;B];

% Observer gain L computation
polesL = [-0.003 0.0006715 -0.00003715]; % desired poles of observer
p = poly(polesL);                        % polynomial coeficients
I = eye(size(A));
phy = p(4)*I+p(3)*A+p(2)*A^2+A^3;
L = phy*inv([C; C*A; C*A^2])*[0 0 1]';   % Ackermann's formula for observer

% Pole-placement gain K for augmented system
polesK = [0.985 0.5353 0.826 0.9581];       % desired poles of the system
p = poly(polesK);                           % polynomial coeficients
I = eye(size(Ai));
phy = p(5)*I+p(4)*Ai+p(3)*Ai^2+p(2)*Ai^3+Ai^4;
K = [0 0 0 1]*inv([Bi Ai*Bi Ai^2*Bi Ai^3*Bi])*phy; % Ackermann's formula for gain K
eig(Ai-Bi*K)

%% Model properties, stability and responses

% MagnetoShield discrete model with desired poles
A_lq = [1        -C; 
     -B*K(1)  A-B*K(2:4)];
B_lq = [1;0;0;0];                   % input is reference r(k)
C_lq = [zeros(size(C,1),1) C];      % measured state - position
D_lq = zeros(size(C,1),1);
sys = ss(A_lq,B_lq,C_lq,D_lq,Ts);

% Properties of the model
Stability = isstable(sys)
Observable_matrix = obsv(sys);      % Observability matrix
Unobservable_states = rank(A_lq)-rank(Observable_matrix)
Control_matrix = ctrb(sys);         % Controlability matrix
Uncontrolable_states = rank(A_lq)-rank(Control_matrix)

% Responses and root locus
figure('Name','Impulse response')
impulse(sys);
figure('Name','Step response')
step(sys);
figure('Name','Root locus')
rlocus(sys)
zgrid
axis([-1.5 1.5 -1.5 1.5])