%   FloatShield grey-box linear transfer function identification procedure
% 
%   Identification of a linear second order transfer function with an
%   integrator, based on measured data.
%
%   The example takes input-output identification results from the
%   FloatShield device, where input is power to the fan in percents and
%   output is the altitude (height) of the ball. Based on the model of
%   Chacon et al. (2017) a first order transfer funciton with an integrator
%   represents ball dynamics, while a first order transfer funciton
%   expresses the relation of air velocity and fan input.
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


clc; clear;                         % Clear screen and memory 

%% Data preprocessing
load resultID.mat                   % Read identificaiton results
data=iddata(y,u,0.025)              % Create identification data object
data = data([4532:5150]);           % Select a relatively stable segment
data = detrend(data);               % Remove offset and linear trends

%% Model configuration
model = idproc('P2I');              % Second order with integrator

model.Structure.Kp.Value=7.8;       % Initial gain
model .Structure.Kp.Maximum=10;     % Maximal gain
model.Structure.Kp.Minimum=5;       % Minimal gain

model.Structure.Tp1.Value=0.3;      % Initial time constant
model.Structure.Tp1.Maximum=0.5;    % Maximal time constant
model.Structure.Tp1.Minimum=0;      % Minimal time constant

model.Structure.Tp2.Value=0.1;      % Initial time constant
model.Structure.Tp2.Maximum=0.3;    % Maximal time constant
model.Structure.Tp2.Minimum=0.1;    % Minimal time constant

%% Estimation configuration
Opt = procestOptions;               % Estimation options
Opt.InitialCondition = 'estimate';  % Estimate the initial condition
Opt.DisturbanceModel = 'ARMA2';     % Define noise model
Opt.Focus = 'stability';            % Goal is to create a stable model
Opt.Display = 'full';               % Full estimation output

%% Estimation
model = procest(data,model, Opt);   % Estimate a process model

%% Verify and compare model
compare(model,data)                 % Compare model to original data
grid on                             % Turn on grid
model                               % List model parameters