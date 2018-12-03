%   HeatShield system identification example
% 
%   Measures and logs the printer head temperature for
%   system identification purposes.
%
%   This example initializes the HeatShield device, then
%   supplies a step change of input to 40% power to the
%   heating cartridge and plots the response. When the
%   experiment is over, the data is saved to a file.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 18.11.2018.
  
%clc; clear all;                              % Clears screen and all variables
% load resultID
Ts=0.004                                      % [s] Sampling
u=resultID(:,1)+resultID(:,2);
y=resultID(:,3)/1000;

data = iddata(y,u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid Voltage';            % Input name
data.InputUnit =  'V';                          % Input unit
data.OutputName = 'Position';                   % Output name
data.OutputUnit = 'm';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(241:end)                             % Discard the time when magnet is on ground, pick close to linearization point

datad=detrend(data);

% Parameters
m = 0.76E-3;                                    % [kg] Magnet mass
R = 198.3;                                      % [Ohm] Solenoid resistance
L = 0.239;                                      % [H] Solenoid inductance
mu0 = 4*pi*1E-7;                                % [H/m] Permeability of free space
mur = 4000                                      % [-] Ratio for the permeability of the core vs. free space (~2000-6000)
mu = mu0*mur;                                   % [H/m] Permeability of the core
N =  250;                                       % [-] Number of windings (turns)
d = 8E-3;                                       % [m] Solenoid core diameter
A = pi*(d/2)^2;                                 % [m^2] Solenoid core cross-sectional area

K=(mu^2*N^2*A)/(2*mu0);                        % Solenoid constant
h0=data(1).y;                                   % Initial position estimate
dh0=(data(2).y-data(1).y)/Ts;                   % Initial velocity estimate
i0=data(1).u/R;                                 % Initial current estimate

FileName      = 'MagnetoShield_ODE_1';            % File describing the model structure.
Order         = [1 1 3];                        % Model orders [ny nu nx].
Parameters    = {K,R,L};             % Initial parameters.
InitialStates = [h0; dh0; i0];                  % Initial state.
TsC            = 0;                             % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, TsC, ...
                'Name', 'Magnetic Levitation');
model = setinit(nlgr, 'Fixed', true);           % Estimate the initial states.
model.Parameters(1).Minimum=0;                  % Cannot be negative
model.Parameters(2).Minimum=0;                  % Cannot be negative
model.Parameters(3).Minimum=0;                  % Cannot be negative


%% Identify model
opt = nlgreyestOptions;
opt.Display='On' % Show progress
opt.SearchOptions.MaxIterations = 50;           % Maximal number of iterations
opt.EstimateCovariance = true;
opt.SearchMethod = 'lsqnonlin';
model2 = pem(data,model,opt)

