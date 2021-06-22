%   MagnetoShield grey-box linearized state space system
%   identification example of the Alternative model
% 
%   This example takes a measurement from the 
%   MagnetoShield device, where the permanent magnet is
%   tracking predefined setpoint trajectory in closed-loop
%   using LQR feedback. The LQR has been designed with use 
%   of the model "MagnetoShield_Models_Greybox_SS.mat".
%   LQR experiment is used for minimalization of the disturbances
%   during the levitation. This improves accuracy of the identification.
%   The model which is identified here, is based on Jakub Mihalik's thesis,
%   which can be found here:
%   https://github.com/gergelytakacs/AutomationShield/wiki/Publications

%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Jakub Mihalík. 
%   Last updated by: Jakub Mihalík
%   Last update: 15.4.20210.

clc; clear all;                                 % Clears screen and all variables

%% Load data for identification
load ../Identif_Data/Data.mat                           % Load data file
Ts=0.00325;                                     % [s] Sampling
y=y/1000;                                       % [m] Output in meters               
i=I/1000;                                       % [A] Current

%% Parameters and constants
g  = 9.81;
m  = 0.74E-3;                                   % [kg] Magnet mass
R  = 210;                                       % [Ohm] Solenoid resistance
L  = 0.235;                                     % [H] Solenoid inductance
Km = 1.5307e-09;                                % Magnetic constant (rhough estimate)
Ke = Km;

% Linearization points
y0 = mean(y);                                   % [m] Output (position) linearization around setpoint
u0 = mean(u);                                   % [V] Input linearization around setpoint
i0 = mean(i);                                   % [A] Current linearization around setpoint
x01 = y0;
x03 = i0;

% New differential states 
delta_y = y-x01;
delta_i = i-x03;
delta_u = u-u0;

% Initial parameters for generic linearized model
alpha   = (4*Km*x03)/(m*x01^5);             % Linearized parameter guess
beta    = Km/(m*x01^4);                     % Linearized parameter guess
gamma   = Ke/(L*x01^4);                     % Linearized parameter guess
delta   = R/L;                              % Parameter guess
epsilon = 1/L;                              % Parameter gues

%% System identification data object
data = iddata([delta_y delta_i],delta_u,Ts);    % Data file
data = data(1000:end);                          % Discard the time when magnet is on ground, pick close to linearization point                             
dataf = fft(data);                              % Frequency domain

% Initial states
h0=data.y(1,1);                                 % Initial position estimate
dh0=(data.y(2,1)-data.y(1,1))/Ts;               % Initial velocity estimate
ii0=data.y(1,2);                                 % Initial current estimate

%% Construct model
A=[0       1        0;                      % Dybamic matrix initial guess
   alpha   0       -beta
   0       gamma   -delta];     

B=[0; 0; epsilon];                              % Input matrix
C=[1 0 0;                                       % Output matrix
   0 0 1];                                      % Distance and current measured
D=[0; 0];                                       % No feed-through                                            
K = zeros(3,2);                                 % Disturbance
x0=[h0; dh0; ii0];                              % Initial condition
disp('Initial guess:')
sys=idss(A,B,C,D,K,x0,0);                       % Construct state-space representation


% Mark the free parameters
sys.Structure.A.Free=      [0     0    0;   % Free and fixed variables
                            1     1    1;
                            0     1    1];
                        
sys.Structure.A.Minimum =  [0      1    0;   % some parameters have to be negative
                            0    -inf -inf;
                            0      0  -inf];

sys.Structure.A.Maximum =  [inf   inf   inf;   % some parameters have to be positive
                            inf    0     0;
                            inf   inf    0];
                       
sys.Structure.B.Free=  [0 0 1]';               % Free and fixed variables
sys.Structure.C.Free=  false;                   % No free parameters

sys.DisturbanceModel = 'estimate';              % Estimate disturbance model
sys.InitialState = 'estimate';                  % Estimate initial states

%% Set estimation options
Options = ssestOptions;                         % State-space estimation options
Options.Display = 'on';                         % Show progress
Options.Focus = 'simulation';                   % Identification focus
Options.EnforceStability = 0;                   % Unstable model
Options.InitialState = 'estimate';              % Estimate initial condition
%Options.OutputWeight = [100 0;0 1];

%% Estimate and list parameters
disp('Identified model:')
model_f = ssest(dataf,sys,Options)             % Launch estimation procedure

figure()
compare(dataf,model_f)                         % Compare data to model in freq-domain

Lnew = 1/model_f.B(3,1);
Kmnew = -model_f.A(2,3)*m*x01^4;
Kenew = model_f.A(3,2)*Lnew*x01^4;
Cnew = -model_f.A(2,2);
Rnew = -model_f.A(3,3)*Lnew;

new = {'L:',num2str(L), '   ', 'L new:',num2str(Lnew);
    'Km:',num2str(Km), '   ', 'Km new:',num2str(Kmnew);
    'Ke:',num2str(Ke), '   ', 'Ke new:',num2str(Kenew);
    'C:',num2str(0),'   ','C new:',num2str(Cnew);
    'R:',num2str(R),'   ','R new:',num2str(Rnew)};
disp(new);

%% Save identified model
y0 = x01;
i0 = x03;
save('MagnetoShield_Alternative_Model.mat','model_f','y0','i0','u0','Ts')