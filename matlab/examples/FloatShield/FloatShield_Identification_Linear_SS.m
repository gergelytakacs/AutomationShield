%   FloatShield grey-box linear state-space identification procedure
% 
%   Identification of a linear third-order state-space model based on 
%   experimental measurement data.
%
%   The example takes input-output identification results from the
%   FloatShield device, where input is power to the fan in percents and
%   output is the altitude (height) of the ball. Based on the model of
%   Chacon et al. (2017) and assuming a first order fan and air veocity
%   relationship the example identifies a third order linear state-space
%   model.
%  
%   References: 
%
%   Chacon, J., Saenz, J., Torre, L., Diaz, J. M., & Esquembre, F. (2017).
%       Design of a Low-Cost Air Levitation System for Teaching Control 
%       Engineering. Sensors (Basel, Switzerland), 17(10), 2321. 
%       doi:10.3390/s17102321
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 27.8.2019.                            


clc; clear;                                    % Clear screen and memory 

%% Data preprocessing
load resultID.mat                              % Read identificaiton results
Ts=0.025;                                      % Sampling
data=iddata(y,u,Ts)                            % Create identification data object
data = data([4530:5151]);                      % Select a relatively stable segment
data = detrend(data);                          % Remove offset and linear trends
  
%% Model configuration
v0=0;                                          % Initial air velocity estimate
h0=data.y(1,1);                                % Initial ball position estimate
dh0=(data.y(2,1)-data.y(1,1))/Ts;              % Initial ball velocity estimate

K     =  7.78;                                 % Fan gain
Tau1  =  0.6;                                  % Ball time constant
Tau2  =  0.14;                                 % Fan time constant

alpha = 1/Tau2;                                % First unknonwn parameter
beta =  1/Tau1;                                % Second unknown parameter
gamma=  K/Tau2;                                % Third unknown parameter

%  Construct model
A=[-alpha  0    0;                             % Dynamic matrix
    0      0    1;
    beta   0   -beta];
B=[gamma; 0; 0];                               % Input matrix
C=[0 1 0];                                     % Output matrix
D=[0];                                         % No feed-through                                            
K = zeros(3,1);                                % Disturbance
K(2) = 25;                                     % State noise
x0=[v0; h0; dh0];                              % Initial condition
disp('Initial guess:')                         % Display
sys=idss(A,B,C,D,K,x0,0)                       % Construct state-space representation

% Mark the free parameters
sys.Structure.A.Free=  [1     0    0;          % Free and fixed variables
                        0     0    0;
                        1     0    1];        
sys.Structure.B.Free=  [1     0    0]';        % Free and fixed variables
sys.Structure.C.Free=  false;                  % No free parameters

sys.DisturbanceModel = 'estimate';             % Estimate disturbance model
sys.InitialState = 'estimate';                 % Estimate initial states

%% Set estimation options
Options = ssestOptions;                        % State-space estimation options
Options.Display = 'on';                        % Show progress
Options.Focus = 'simulation';                  % Identification focus on simulation
Options.EnforceStability = 1;                  % Stable model
Options.InitialState = 'estimate';             % Estimate initial condition

%% Estimate and list parameters
disp('Identified model:')
model = ssest(data,sys,Options);               % Launch estimation procedure
compare(data,model);                           % Compare data to model
model                                          % List model parameters
grid on                                        % Turn on grid