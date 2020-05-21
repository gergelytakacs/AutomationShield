%   MotoShield grey-box identification procedure
% 
%   Identification of a grey-box model using experimental data.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács and Ján Boldocký.

clc; clear; close all;

%% Data preprocessing
load aprbsResult.mat                            % Read identificaiton results
Ts = 0.02;                                      % Sampling Period
data = iddata(y,u,Ts,'Name','Experiment');      % Create identification data object
data = detrend(data);                           % Remove offset and linear trends
data.InputName = 'Input Voltage';               %_Input
data.InputUnit =  'V';                          %_Input unit
data.OutputName = 'Angular Velocity';           %_Output
data.OutputUnit = 'rad/s';                      %_Output unit
data.Tstart = 0;                                %_Time offset
data.TimeUnit = 's';                            %_Time unit
%% Initial guess of model parameters
J = 0.01;       %_Moment of inertia
b = 0.1;        %_Viscous friction constant
Ke = 0.007;     %_Electromotive force constant # Voltage over Velocity
Kt = 0.007;     %_Motor torque constant # Torque over Current
R = 10;         %_Electric resistance
L = 0.5;        %_Electric inductance

dtheta0  = data.y(1,1);     %_Initial angular velocity

% TF Model
   
b0 = Kt;                         %_Gain # Numerator
a2 =  J*L;                       %_Polynomial coefficients # Denominator
a1 = (J*R+b*L);
a0 = (b*R+Kt^2);

disp('Initial model:')
sys = idtf([b0],[a2 a1 a0],Ts)                   %_Initial guess of the model

%Configure estimation procedure
Options = tfestOptions;                         % Identification options for tfest
Options.Display = 'on';                         % Show progress
Options.EnforceStability = 1;                   % Unstable models allowed
%Options.InitialCondition = 'estimate';         % Estimate initial condition as well

%Identify model
disp('Estimated model:')                       
model = tfest(data,sys,Options)                  % Identification result model

compare(data,model);                           % Compare data to model
model                                          % List model parameters
grid on                                        % Turn on grid

return
save MotoShield_GreyboxModel_TF model