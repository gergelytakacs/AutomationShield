%   HeatShield grey-box identification procedure
% 
%   Identification of a first-principles model-based grey box model
%   based on experimental data.
%
%   This example takes measurement data from the HeatShield, then
%   uses the differential equation in "HeatShield_ODE.m" and initial guesses
%   for its parameters to create a model. This model is then fitted to 
%   measurement data using an adaptive Newton-Gauss optimization procedure.
%   The model is saved as a file. Note that the optimization procedure may
%   take a while to run.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 18.10.2018.

clc; clear;

% result=readExperiment(8100,'COM22',9600);
% save resultID result

load resultID2                                  % Load measurement data

%% Create system identification data object
Ts=2;                                           % [s] Sampling
Y=resultID(:,1);                                % [oC] Output
U=resultID(:,2)/100;                            % Duty cycle of input
data = iddata(Y,U,Ts,'Name','Printer Head');    % Data file
data.InputName = 'Duty cycle';                  % Input name
data.InputUnit =  'Tp/Td';                      % Input unit
data.OutputName = 'Temperature';                % Output name
data.OutputUnit = '^oC';                        % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit

%% Initial guess of model parameters
m = 0.010;                                      % [kg] Weight of the block
c = 897;                                        % [J/K/kg]Specific heat of aluminum
R = 18;                                         % [Ohm] Resistance of the cartridge
h = 20;                                         % [W/(m2K)] Convective heat transfer coefficient
A = 1504E-6;                                    % [m2] Area of the heating block
V = 6.45;                                       % [V] Amplitude of the input voltage

alpha     = h*A;                                % Heat convection "resistance" 
beta      = m*c;                                % Heat capacity "capacitance"
gamma     = V^2/R;                              % PWM signal power
Ta     = data.Outputdata(1);                    % Ambient temperature

%% Model structure
FileName      = 'HeatShield_ODE';                  % File describing the model structure.
Order         = [1 1 1];                        % Model orders [ny nu nx].
Parameters    = {alpha,beta,gamma,Ta};          % Initial parameters. Np = 2.
InitialStates = Ta;                             % Initial initial states.
Ts            = 0;                              % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
                'Name', 'Printer Head');
model = setinit(nlgr, 'Fixed', true);           % Estimate the initial states.

%% Identify model
opt = nlgreyestOptions('Display', 'on','EstimateCovariance',true,'SearchMethod','gna');
opt.SearchOptions.MaxIterations = 50;           % Maximal number of iterations
model = nlgreyest(data, nlgr, opt);             % Run identification procedure
compare(data, model);                           % Compare data with model
save HeatShield_Models_Greybox.mat model        % Save results

  