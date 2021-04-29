%   MAGNETOSHIELD EXPLICIT EXPORT EXAMPLE
%
%   This example reads the linearized alternative model of the 
%   MagnetoShield device, then expands it with an integrator and then 
%   computes an explicit model predictive controller (EMPC, Explicit MPC) 
%   using the Multi-Parametric Toolbox of Kvasnica et al. The code has been 
%   tested with v. 3.0 of the MPT. Please see https://www.mpt3.org/ for the 
%   free download and installation instruction. The code exports the 
%   controller into a C language code. To use this with our examples, 
%   please note that
%   - AVR architecture Arduinos are too slow for this example
%   - DUE works well with this example
%   - change all "static" variables to "const"
%   - remove the sequential evaluation function from the end.
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
%   Created by:       Gergely Takács
%   Created on:       9.11.2020
%   Last updated by:  Jakub Mihalík
%   Last update on:   15.15.2021

clc; clear; close all;                                          % Close and clear all

load '../../Identification/Linear/MagnetoShield_Alternative_Model.mat';
                                                               
Ts=0.003;                                                       % [s] sampling for discrete control
N=2;                                                            % [steps] prediction horizon
ul=0;                                                           % [V] Adjust lower input constraint for linearization
uh=10;                                                          % [V] Adjust upper input constraint for linearization


ul=ul-u0;                                                       % [V] Adjust lower input constraint for linearization
uh=uh-u0;                                                       % [V] Adjust upper input constraint for linearization

%% Model discretization
modeld=c2d(model_f,Ts);                                           % Discretized linear state-space model
A=modeld.a;                                                     % Extract A
B=modeld.b;                                                     % Extract B
C=[1 0 0];                                                      % C for introducing an integration component    

%% Model augmentation by the integrator
[ny nx]=size(C);                                                % Sizing C
[nx nu]=size(B);                                                % Sizing B
Ai=[eye(ny,ny) -C;                                              % Augmenting A by an integrator
    zeros(nx,ny)  A];          
Bi=[zeros(ny,nu); B];                                           % Augmenting B by the integrator
Ci=[zeros(ny,ny) C];                                            % Augmenting C by the integrator

%% MPC Penalty
% Qmpc=diag([50 100 100 10]);                                     % State penalty matrix
% Rmpc=0.1;                                                       % Input penalty matrix
Qmpc=diag([70 100 20 40]);                                     % State penalty matrix
Rmpc=0.01;                                                       % Input penalty matrix


%% Model and EMPC controller using the MPT 3.0
model = LTISystem('A', Ai, 'B', Bi, 'C', Ci, 'Ts', Ts);         % LTI model by the MPT 3.0
model.u.min = [ul];                                             % [V] Lower input constraint for the MPT
model.u.max = [uh];                                             % [V] Upper input constraint for the MPT
model.x.penalty = QuadFunction(Qmpc);                           % state penalty for the MPT
model.u.penalty = QuadFunction(Rmpc);                           % input penalty for the MPT
PA = model.LQRPenalty;                                          % compute LQ penalty by the MPT
model.x.with('terminalPenalty');                                % enable terminal penalty in the MPT
model.x.terminalPenalty = PA;                                   % define for the MPT

%% Compute controller
ctrl = MPCController(model, N)           % create an MPC controller by the MPT
ectrl = ctrl.toExplicit();               % transform it to EMPC by the MPT

%% Export controller
ectrl.exportToC('ectrl','empc')          % Export to Simulink mex function
cd empc;    
mex ectrl_sfunc.c;

%% Not needed for use in Simulink 
% empcToC(ectrl,'generic');                                       % Export to code suitable for Due, does not work on AVR
% empcToPython(ectrl);                                            % Export to code suitable for Python
% 
% empcMemory(ectrl,'float')                                       % This many bytes for the optimizer matrices
