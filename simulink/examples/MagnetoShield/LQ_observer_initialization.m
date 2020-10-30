clear;close all;clc;
load '../../../matlab/examples/MagnetoShield/MagnetoShield_Models_Greybox_SS';
%% MagnetoShield - LQR simulation and control initialization for Simulink
Ts = 0.002%0.001;                   % Sampling of the Simulink simulation

% Matrices extraction from model
modeld=c2d(model,Ts);           % Discretized linear state-space model
A = modeld.a;                   % Extract A
B = modeld.b;                   % Extract B
C = [1 0 0];                   % Position is measured for integrator 'xi' computation
D = 0;

%%
% L gain computation
poles = [-0.003 0.0006715 -0.00003715];
p = poly(poles);
I = eye(size(A));
phy = p(4)*I+p(3)*A+p(2)*A^2+A^3;
L = phy*inv([C; C*A; C*A^2])*[0 0 1]';
eig(A-L*C);
%%
Ai = [1 -C;
      zeros(3,1) A];
Bi = [0;B];


% LQ gain computation - the 1. tuning diverges in the simulations but is 
% smoother in experiments:
%Experiments
Qlq=diag([30 180 120 60]);      % State penalty matrix
Rlq=0.0001;                       % Input penalty matrix
K=dlqr(Ai,Bi,Qlq,Rlq);         % LQ gain for system states
% 
% %Simulations
% Qlq=diag([70 100 20 40]);      % State penalty matrix
% Rlq=0.1;                       % Input penalty matrix
% K=dlqr(Ai,Bi,Qlq,Rlq);         % LQ gain for system states

%%
% sys = [A-B*K B*K; zeros(3,3) A-L*C];
% eig(sys)
% %%
% % Discrete augmented model with integrator state - unstable, unobservable
% Ai = [1 -Ci;
%       zeros(3,1) A];
% Bi = [0;B];
% 
% % LQ gain computation
% Qlq=diag([70 100 20 40]);      % State penalty matrix
% Rlq=0.1;                       % Input penalty matrix
% K=dlqr(Ai,Bi,Qlq,Rlq);         % LQ gain for system states
% 
% 
% % Measurement matrices of state-space and Kalman filter gains
% if order == 'SISO'                  % Setup if just position is measured
%     C = [1 0 0];
%     D = 0;
%     Q = diag([0.0001, 100, 100]);   % Penalty of the model for Kalman filter
%     R = 0.001;                     % Penalty of measurement for Kalman filter
%     
% elseif order == 'SIMO'             % Setup if position and current are measured
%     C = [1 0 0;0 0 1];
%     D = [0;0];
%     Q = diag([0.0001, 100, 80]);   % Penalty of the model for Kalman filter
%     R = [0.001, 0.01];             % Penalty of measurement for Kalman filter
% end
% %% Model properties, stability and responses
% 
% % MagnetoShield discrete model with LQ-gain implemented
% A_lq = [A-B*K(2:4) -B*K(1); 
%           -Ci          1   ];
% B_lq = [0;0;0;1];                   % input is reference r(k)
% C_lq = [1 0 0 0];                   % measured state - position
% D_lq = [0];
% sys = ss(A_lq,B_lq,C_lq,D_lq,Ts);
% 
% % Properties of the model
% Stability = isstable(sys)
% Observable_matrix = obsv(sys);      % Observability matrix
% Unobservable_states = rank(A_lq)-rank(Observable_matrix)
% Control_matrix = ctrb(sys);         % Controlability matrix
% Uncontrolable_states = rank(A_lq)-rank(Control_matrix)
% 
% % Responses and root locus
% figure('Name','Impulse response')
% impulse(sys);
% figure('Name','Step response')
% step(sys);
% figure('Name','Root locus')
% rlocus(sys)
% zgrid
% axis([-1.5 1.5 -1.5 1.5])