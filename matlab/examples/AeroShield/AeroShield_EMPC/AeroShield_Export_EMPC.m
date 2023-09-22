%   AEROSHIELD EXPLICIT EXPORT EXAMPLE
%
%   This example reads the linearized model of the AeroShield device,
%   then expands it with an integrator and then computes an explicit model
%   predictive controller (EMPC, Explicit MPC) using the Multi-Parametric
%   Toolbox of Kvasnica et al. The code has been tested with v. 3.0 of the
%   MPT. Please see https://www.mpt3.org/ for the free download and
%   installation instruction. The code exports the controller into a C
%   language code. To use this with our examples, please note that
%   - for AVR architecture Arduinos use 'AVR' in "empcToC()" 
%       (UNO does not have enaugh memory for contoller with prediction
%       horizon larger than 3)
%   - for DUE and other ARM-based boards use 'generic' in "empcToC()"
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   If you have found any use of this code, please cite our work in your
%   academic publications, such as thesis, conference articles or journal
%   papers. A list of publications connected to the AutomationShield
%   project is available at: 
%   https://github.com/gergelytakacs/AutomationShield/wiki/Publications
%
%   Created by:       Gergely Takács and Erik Mikuláš
%   Created on:       8.8.2023
%   Last updated by:  Erik Mikuláš
%   Last update on:   22.9.2023
%


startScript;                                            % Clears screen and variables, except allows CI testing

load ../AeroShield_GreyboxModel_Linear                  % Include linearized state-space model
                                                               
Ts=0.01;                                                % [s] Sampling for discrete control
N=5;                                                    % [steps] Prediction horizon
ul=0;                                                   % [V] Adjust lower input constraint
uh=3.7;                                                 % [V] Adjust upper input constraint


%% Model discretization
modeld=c2d(linsys,Ts);                                  % Discretized linear state-space model
A=modeld.a;                                             % Extract A
B=modeld.b;                                             % Extract B
C=[1 0];                                                % C for introducing an integration component    

%% Model augmentation by the integrator
[ny, ~]=size(C);                                        % Sizing C
[nx, nu]=size(B);                                       % Sizing B
Ai=[eye(ny,ny) -C;                                      % Augmenting A by an integrator
    zeros(nx,ny)  A];          
Bi=[zeros(ny,nu); B];                                   % Augmenting B by the integrator
Ci=[zeros(ny,ny) C];                                    % Augmenting C by the integrator

%% MPC Penalty
Qmpc=diag([5 1 100]);                                   % State penalty matrix
Rmpc=2e3;                                               % Input penalty matrix

%% Model and EMPC controller using the MPT 3.0
model = LTISystem('A', Ai, 'B', Bi, 'C', Ci, 'Ts', Ts); % LTI model by the MPT 3.0
model.u.min = [ul];                                     % [V] Lower input constraint for the MPT
model.u.max = [uh];                                     % [V] Upper input constraint for the MPT
model.x.penalty = QuadFunction(Qmpc);                   % State penalty for the MPT
model.u.penalty = QuadFunction(Rmpc);                   % Input penalty for the MPT
PA = model.LQRPenalty;                                  % Compute LQ penalty by the MPT
model.x.with('terminalPenalty');                        % Enable terminal penalty in the MPT
model.x.terminalPenalty = PA;                           % Define for the MPT

%% Compute controller
ctrl = MPCController(model, N)                          % Create an MPC controller by the MPT
ectrl = ctrl.toExplicit();                              % Transform it to EMPC by the MPT

%% Export controller
%ectrl.exportToC('ectrl','empc')                        % Export to Simulink mex function
%    cd empc;    
%    mex ectrl_sfunc.c;

empcToC(ectrl,'AVR');                                   % Export to code, for ARM MCU use 'generic', for AVR use 'AVR'
empcToPython(ectrl);                                    % Export to code suitable for Python

empcMemory(ectrl,'float')                               % This many bytes for the optimizer matrices