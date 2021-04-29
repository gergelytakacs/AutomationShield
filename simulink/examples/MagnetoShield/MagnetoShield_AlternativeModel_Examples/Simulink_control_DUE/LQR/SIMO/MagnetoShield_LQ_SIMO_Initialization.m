%   MAGNETOSHIELD LQR SIMO simulation and control initialization for Simulink
%
%   Code below is used as an initialization script for MagnetoShield system
%   and its SIMO regulation. Default properties can be changed via this 
%   script.
%   This example reads the linearized alternative model of the 
%   MagnetoShield device, then expands it with an integrator and then 
%   computes LQ gain based on the states and input penalty matrices. The
%   model assumes two reference signals. One for position and another one
%   for current.
%   Kalman filter uses measurements of both measurable states.
%   At the end are the model properties as stability, controlability,
%   observability, step and impulse response evaluated and displayed. 
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
%   Created on:       15.4.2021
%   Last updated by:  Jakub Mihalik
%   Last update on:   15.4.2021

clear;close all;clc;
load '../../../Identification'/Linear/MagnetoShield_Alternative_Model.mat;
%% MagnetoShield - LQR simulation and control initialization for Simulink

Ts = 0.0015;                   % Sampling of the Simulink simulation

% Matrices extraction from model
modeld=c2d(model_f,Ts); % Discrete linear state-space model
A = modeld.a;           % Extract A
B = modeld.b;           % Extract B
C = modeld.c;           % Extract C - measures position and current too
D = modeld.d;

% Discrete augmented model with integrator states
% We have 2 integrator states 2 references now - for position and current
Ai = [A zeros(3,2);
     -C eye(2)];
Bi = [B;0;0];
Ci = [1 0 0 0 0; 0 0 1 0 0];
Di = [0;0];
sysi = ss(Ai,Bi,Ci,Di,Ts);

% SIMO
Qlq=diag([100 20 40 20 10]);        % State penalty matrix 
Rlq = 0.1;                          % Input penalty matrix
K=dlqr(Ai,Bi,Qlq,Rlq);              % LQ gain for system states
Ck = [1 0 0;0 0 1];
Dk = [0;0];
Q = diag([0.0001 , 100 , 100]);     % Kalman filter matrices
R = diag([1.5815e-10,6.6217e-09]);


%% Model properties, stability and responses
% These are properties of the stabilized model with LQ gain implemented.
% To see the properties of the original model, substitute 'sys'->'sysi'.

% MagnetoShield discrete model with LQ-gain implemented
% Inputs are now references - so we have two inputs in this form!
A_lq = [A-B*K(1:3) -B*K(4:5); 
       -C          eye(2)];
B_lq = [0 0;0 0;0 0;1 0;0 1];       % inputs are references
C_lq = [C zeros(size(C,1),2)];      % measured states - position and current
D_lq = [0 0;0 0];
sys = ss(A_lq,B_lq,C_lq,D_lq,Ts);
pole(sys)

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
