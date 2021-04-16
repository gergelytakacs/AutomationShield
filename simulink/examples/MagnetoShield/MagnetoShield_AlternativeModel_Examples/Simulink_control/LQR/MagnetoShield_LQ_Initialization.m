%   MAGNETOSHIELD LQR simulation and control initialization for Simulink
%
%   Code below is used as an initialization script for MagnetoShield system
%   and default properties can be changed via this script.
%   This example reads the linearized alternative model of the 
%   MagnetoShield device, then expands it with an integrator and then 
%   computes LQ gain based on the states and input penalty vectors. Based 
%   on the dimension of the measured signal is Kalman filter tuned as well.
%   'S1' - measured position, 'S2' - measured current and position.
%   Consequently are the model properties as stability, controlability,
%   observability, step and impulse response evaluated and displayed. 
%   In the case of single state feedback is root-locus plot shown too.
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
load '../../Identification/Linear/MagnetoShield_Alternative_Model.mat';
%% MagnetoShield - LQR simulation and control initialization for Simulink

Ts = 0.0015;                   % Sampling of the Simulink simulation

% How many states are measured ('S1', 'S2')
SStates = 'S1';

% Matrices extraction from model
modeld=c2d(model_f,Ts);             % Discretized linear state-space model
A = modeld.a;                       % Extract A
B = modeld.b;                       % Extract B
C = [1 0 0];                        % Position is measured for integrator 'xi' computation
D = 0;

% Discrete augmented model with integrator state - unstable, unobservable
Ai = [A zeros(3,1);
     -C 1];
Bi = [B;0];
Ci = [1 0 0 0];
Di = 0;
sysi = ss(Ai,Bi,Ci,Di,Ts);

if(SStates=='S1')
% One state for Kalman
    % LQ gain computation
    Qlq=diag([100 20 40 70]);       % State penalty matrix
    Rlq=0.01;                       % Input penalty matrix
    K=dlqr(Ai,Bi,Qlq,Rlq);          % LQ gain for system states
    Q = diag([0.0001 , 100 , 100]); % Penalty of the model for Kalman filter
    R = diag(1.5815e-10);
    Ck = [1 0 0];
    Dk = [0];
elseif(SStates=='S2')
% 2 states for Kalman
    Qlq=diag([100 20 40 70]);       % State penalty matrix
    Rlq = 0.01;                     % Input penalty matrix
    K=dlqr(Ai,Bi,Qlq,Rlq);          % LQ gain for system states
    Q = diag([0.0001 , 100 , 100]); % Penalty of the model for Kalman filter
    R = diag([1.5815e-10,6.6217e-09]);
    Ck = [1 0 0;0 0 1];
    Dk = [0;0];
end
%% Model properties, stability and responses
% These are properties of the stabilized model with LQ gain implemented.
% To see the properties of the original model, substitute 'sys'->'sysi'.

% MagnetoShield discrete model with LQ-gain implemented
% In matrix 'A_lq' is used 'C' not 'Ck' because 'Ck' denotes measured 
% states, 'C' denotes states feededback for augmented form.
A_lq = [A-B*K(1:3) -B*K(4); 
       -C           ones(size(C,1),1)];
B_lq = [0;0;0;1];                   % input is reference r(k)
C_lq = [Ck zeros(size(Ck,1),1)];      % measured state - position
D_lq = zeros(size(Ck,1),1);
sys = ss(A_lq,B_lq,C_lq,D_lq,Ts);
pole(sys)

% Properties of the model
Stability = isstable(sys)
Observable_matrix = obsv(sys);      % Observability matrix
Unobservable_states = rank(A_lq)-rank(Observable_matrix)
Control_matrix = ctrb(sys);         % Controlability matrix
Uncontrolable_states = rank(A_lq)-rank(Control_matrix)

% Responses
figure('Name','Impulse response')
impulse(sys);
figure('Name','Step response')
step(sys);                  

% Root locus plot - possible for S1 only
if(SStates=='S1')
    figure('Name','Root locus')
    rlocus(sys)
    zgrid
    axis([-1.5 1.5 -1.5 1.5])
end