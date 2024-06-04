clear all; close all; clc;
%%
load BicopShield_GreyboxModel_LinearSS.mat                  % Include linearized state-space model
                                                               
Ts=0.01;                                                % [s] Sampling for discrete control
N=5;                                                    % [steps] Prediction horizon
ul=[-30;-30];                                                   % [V] Adjust lower input constraint
uh=[30;30];                                                 % [V] Adjust upper input constraint


%% Model discretization
modeld=c2d(model,Ts);                                  % Discretized linear state-space model
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
Qmpc=diag([1 10 1]);                     % State penalty matrix
Rmpc=diag([1e-2, 1e-2]);                                               % Input penalty matrix

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
ectrl.exportToC('ectrl','empc')                        % Export to Simulink mex function
   cd empc;    
   mex ectrl_sfunc.c;

empcToC(ectrl,'AVR');                                   % Export to code, for ARM MCU use 'generic', for AVR use 'AVR'
empcToPython(ectrl);                                    % Export to code suitable for Python

empcMemory(ectrl,'float')                               % This many bytes for the optimizer matrices