clear;close all;clc;
load 'MagnetoShield_Models_Greybox_SS model.mat';
%% MagnetoShield - LQR simulation and control initialization for Simulink

order = 'SISO';                 % SISO - position / SIMO - position & current
Ts = 0.0025;                     % Sampling

% Matrices extraction from model
modeld=c2d(model,Ts);           % Discretized linear state-space model
A = modeld.a;                   % Extract A
B = modeld.b;                   % Extract B
Ci = [1 0 0];                   % Position is measured for integrator 'xi' computation
D = 0;

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
A_lq = [A-B*K(2:4) -B*K(1); 
          -Ci          1   ];
B_lq = [0;0;0;1];                   % input is reference r(k)
C_lq = [1 0 0 0];                   % measured state - position
D_lq = [0];
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
figure('Name','Root locus')
rlocus(sys)
zgrid
axis([-1.5 1.5 -1.5 1.5])