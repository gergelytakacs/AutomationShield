

startScript;                                    % Clears screen and variables, except allows CI testing

load LinkShield_ID_Data.mat                  % Load data file
Ts=0.005;                                    % [s] Sampling


%% Parameters


B_eq = 0.004;
J_eq = 0.00208;
NI_m = 0.69;
NI_g = 0.90;
K_m  = 0.00768;
K_g  = 70;
R_m  = 2.6;
K_s  = 1.4;

K_t  = 1;
J_link = 1;

y = y';
u = u';


% Initial parameters for generic linearized model
alpha   = K_s/J_eq;                 
beta    = -(NI_g*(K_g^2)*NI_m*K_t*K_m+B_eq*R_m)/(R_m*J_eq);                  
gamma   = -(K_s*(J_eq+J_link))/(J_eq*J_link);
delta   = (NI_g*(K_g^2)*NI_m*K_t*K_m+B_eq*R_m)/(R_m*J_eq);                  
epsylon = (NI_g*K_g*NI_m*K_t)/(R_m*J_eq);
zeta    = -(NI_g*K_g*NI_m*K_t)/(R_m*J_eq);


%% System identification data object
data = iddata(y,u,Ts,'Name','Link Model');    % Data file
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(3990:4200);                           % Discard the time when magnet is on ground, pick close to linearization point
data=detrend(data);                             % Discard offset, takes care of linearization too (if mean is taken as lin. point)
dataf = fft(data);                              % Frequency domain 																		   

% Initial states
Fi0=data.y(1,1);                                 % Initial position estimate
Alpha0=data.y(1,2); 
dFi0=(data.y(2,1)-data.y(1,1))/Ts; 
dAlpha0=(data.y(2,2)-data.y(1,2))/Ts; 
                             
%% Construct model

    A=[0      0        1     0;
       0      0        0     1;
       0      alpha    beta     0;
       0      gamma    delta     0];

B=[  0;  0; epsylon; zeta];                              % Input matrix
C=[1 0 0 0
   0 1 0 0 ];                                       % Output matrix                                      % Distance and current measured
D=[0; 0];                                       % No feed-through                                            
K = zeros(4,2);                                 % Disturbance
x0=[Fi0; Alpha0; dFi0; dAlpha0;];                               % Initial condition
disp('Initial guess:')
sys=idss(A,B,C,D,K,x0,0) ;                       % Construct state-space representation


% Mark the free parameters

    sys.Structure.A.Free=      [0     0    0    0;   % Free and fixed variables
                                0     0    0    0;
                                0     1    1    0;
                                0     1    1    0];

                      
sys.Structure.B.Free=  [0  0 1 1]';               % Free and fixed variables
sys.Structure.C.Free=  false;                   % No free parameters
sys.Structure.D.Free=  [0;0]; 

sys.InitialState = 'estimate';                  % Estimate initial states

%% Set estimation options
Options = ssestOptions;                         % State-space estimation options
Options.Display = 'on';                         % Show progress
Options.Focus = 'simulation';                   % Identification focus
Options.EnforceStability = 0;                   % Unstable model
Options.InitialState = 'estimate';              % Estimate initial condition

%% Estimate and list parameters
disp('Identified model:')
model = ssest(data,sys,Options)                % Launch estimation procedure
compare(data,model,1)                            % Compare data to model
