clc;
clear;

load MagnetoShield_Models_Greybox_SS

dt = 0.05;                                                      % [s] Sample time for discretization
N  = 5;                                                         % [steps] Target horizon

umin = 0;                                                       % [V] minimal input voltage
umax = 10;                                                      % [V] maximal input voltage

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
Q=diag([1E2 10 10 1]);                                        % State penalty matrix
R=1E-3;                                                       % Input penalty matrix

muAOMPC_Problem('MagnetoShield', Ad, Bd, Q, R, dt, N, umin, umax)
muAOMPC_Main('MagnetoShield')