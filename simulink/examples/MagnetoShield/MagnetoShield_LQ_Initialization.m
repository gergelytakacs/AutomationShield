%   MAGNETOSHIELD LQR simulation and control initialization for Simulink
%
%   Code below is used as an initialization script for MagnetoShield system
%   and default properties can be changed via this script.
%   This example reads the linearized model of the MagnetoShield device,
%   then expands it with an integrator and then computes LQ gain based on
%   the states and input penalty vectors. Based on the dimension of the
%   feedback signal is Kalman filter tuned too ('SISO' - feedbac just 
%   position, 'SIMO' - feedbac current and position).
%   Consequently are the model properties as stability, controlability,
%   observability, step and impulse response evaluated and displayed. 
%   In the case of 'SISO' feedback is root-locus plot shown too.
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

%% MagnetoShield - LQR simulation and control initialization for Simulink

order = 'SISO';                 % SISO - position / SIMO - position & current
Ts = 0.001;                    % Sampling of the Simulink simulation

% Matrices extraction from model
modeld=c2d(model,Ts);           % Discretized linear state-space model
A = modeld.a;                   % Extract A
B = modeld.b;                   % Extract B
Ci = [1 0 0];                   % Position is measured for integrator 'xi' computation

% Discrete augmented model with integrator state - unstable, unobservable
Ai = [1 -Ci;
      zeros(3,1) A];
Bi = [0;B];

% LQ gain computation
Qlq=diag([70 100 20 40]);      % State penalty matrix
Rlq=0.1;                       % Input penalty matrix
K=dlqr(Ai,Bi,Qlq,Rlq);         % LQ gain for system states

% Measurement matrices of state-space and Kalman filter gains
if order == 'SISO'                  % Setup if just position is measured
    C = [1 0 0];
    D = 0;
    Q = diag([0.0001, 100, 100]);   % Penalty of the model for Kalman filter
    R = 0.001;                     % Penalty of measurement for Kalman filter
    
elseif order == 'SIMO'             % Setup if position and current are measured
    C = [1 0 0;0 0 1];
    D = [0;0];
    Q = diag([0.0001, 100, 80]);   % Penalty of the model for Kalman filter
    R = [0.001, 0.01];             % Penalty of measurement for Kalman filter
end

%% Model properties, stability and responses

% MagnetoShield discrete model with LQ-gain implemented
A_lq = [1        -Ci; 
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
if order=='SISO'                    % root locus plot possible for SISO
    figure('Name','Root locus')
    rlocus(sys)
    zgrid
    axis([-1.5 1.5 -1.5 1.5])
end