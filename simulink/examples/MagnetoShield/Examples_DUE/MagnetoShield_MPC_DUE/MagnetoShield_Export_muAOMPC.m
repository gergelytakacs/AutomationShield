%   MagnetoShield muAO-MPC Python problem definition

%   This example reads the linearized model of the MagnetoShield device,
%   then creates a problem definition file and a main file in Python for
%   the muAO-MPC model predictive control (MPC) software. The resulting 
%   Python files shall be run by the users according to the instructions
%   supplied with muAO-MPC. The resulting computationally and memory
%   efficient code was tested with an Arduino atMega 2560, see the 
%   MagnetoShield_MPC example.

%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by:  Gergely Takács. 
%   Created on:  23.9.2020
%   Last updated by:  Jakub Mihalik
%   Last update: 5.12.2020

clc;
clear;

load '../../../../../matlab/examples/MagnetoShield/MagnetoShield_Models_Greybox_SS';

dt = 0.003;                                                      % [s] Sample time for discretization
N  = 7;                                                         % [steps] Target horizon

umin = 0-u0;                                                    % [V] minimal input voltage
umax = 10-u0;                                                   % [V] maximal input voltage

%% Discretization
modeld=c2d(model,dt);                                           % Discretized linear state-space model
A=modeld.a;                                                     % Extract A
B=modeld.b;                                                     % Extract B
C=[1 0 0];                                                      % C for introducing an integration component

%% Model augmentation by integration
[ny nx]=size(C);                                                % Sizing C
[nx nu]=size(B);                                                % Sizing B
Ad=[eye(ny,ny) -C;                                              % Augmenting A by an integrator
    zeros(nx,ny)  A];          
Bd=[zeros(ny,nu); B];                                           % Augmenting B by the integrator
Cd=[zeros(ny,nx) C];                                            % Augmenting C by the integrator

%% LQ design
Q=diag([200 1000 100 10]);                                        % State penalty matrix
R=0.01;                                                       % Input penalty matrix

muAOMPC_Problem('MagnetoShield', Ad, Bd, Q, R, dt, N, umin, umax)
muAOMPC_Main('MagnetoShield')