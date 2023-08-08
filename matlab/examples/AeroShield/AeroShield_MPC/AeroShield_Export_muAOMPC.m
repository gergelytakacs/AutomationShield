%   AeroShield muAO-MPC Python problem definition

%   This example reads the linearized model of the AeroShield device,
%   then creates a problem definition file and a main file in Python for
%   the muAO-MPC model predictive control (MPC) software. The resulting 
%   Python files shall be run by the users according to the instructions
%   supplied with muAO-MPC. The resulting computationally and memory
%   efficient code was tested with an Arduino atMega 2560, see the 
%   AeroShield_MPC example.

%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by:  Gergely Takács and Erik Mikuláš. 
%   Created on: 08.08.2023
%   Created on: 08.08.2023

startScript;                                    % Clears screen and variables, except allows CI testing
load ../AeroShield_GreyboxModel_Linear

dt = 0.015;                                                     % [s] Sample time for discretization
N  = 7;                                                         % [steps] Target horizon

umin = 0;                                                    % [V] minimal input voltage
umax = 3.7;                                                  % [V] maximal input voltage

%% Discretization
modeld=c2d(linsys,dt);                                          % Discretized linear state-space model
A=modeld.a;                                                     % Extract A
B=modeld.b;                                                     % Extract B
C=[1 0];                                                        % C for introducing an integration component

%% Model augmentation by integration
[ny, ~]=size(C);                                                % Sizing C
[nx, nu]=size(B);                                                % Sizing B
Ad=[eye(ny,ny) -C;                                              % Augmenting A by an integrator
    zeros(nx,ny)  A];          
Bd=[zeros(ny,nu); B];                                           % Augmenting B by the integrator
Cd=[zeros(ny,nx) C];                                            % Augmenting C by the integrator

%% LQ design
Q=diag([50 100 1]);                                        % State penalty matrix
R= 3e3;                                                       % Input penalty matrix

muAOMPC_Problem('AeroShield', Ad, Bd, Q, R, P, N, nx+ny, nu, umin, umax)
muAOMPC_Main('AeroShield')

!python AeroShield_muAOMPC_Main.py