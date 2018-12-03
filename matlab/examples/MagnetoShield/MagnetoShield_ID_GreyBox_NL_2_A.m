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

%data=detrend(data);

% Parameters
m = 0.76E-3;                                    % [kg] Magnet mass
R = 198.3;                                      % [Ohm] Solenoid resistance
L = 0.239;                                      % [H] Solenoid inductance
mu0 = 4*pi*1E-7;                                % [H/m] Permeability of free space
mur = 4000;                                     % [-] Ratio for the permeability of the core vs. free space (~2000-6000)
mu = mu0*mur;                                   % [H/m] Permeability of the core
N =  500;                                       % [-] Number of windings (turns)
d = 8E-3;                                       % [m] Solenoid core diameter
A = pi*(d/2)^2;                                 % [m^2] Solenoid core cross-sectional area

% Physically meaningful estimates
km=(mu^2*N^2*A)/(2*mu0);                        % Magnetic force constant
alpha=km/m;                                     % Parameter 1
beta=1/L;                                       % Parameter 2
gamma=R;                                        % Parameter 3
delta=10;                                       % Parameter 4

return
% Physically meaningful initial values
h0=data(1).y;                                   % Initial position estimate
dh0=(data(2).y-data(1).y)/Ts;                   % Initial velocity estimate
i0=data(1).u/R;                                 % Initial current estimate
%InitialStates = [h0; dh0; i0];                  % Initial state.

%% Manual changes
%alpha=1000;                                      % Parameter 1
%beta=10;                                       % Parameter 2
%gamma=10;                                        % Parameter 3
%delta =1;                                         % Parameter 4
InitialStates = [-0.015; 0; 0];                        % Initial state.

FileName      = 'MagnetoShield_ODE_2';          % File describing the sys structure.
Order         = [1 1 3];                        % Model orders [ny nu nx].
Parameters    = {alpha,beta,gamma,delta};       % Initial parameters.

TsC            = 0;                             % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, TsC, ...
                'Name', 'Magnetic Levitation');
sys = setinit(nlgr, 'Fixed', false);           % Estimate the initial states.
sys.Parameters(1).Minimum=0;                  % Cannot be negative
sys.Parameters(2).Minimum=0;                  % Cannot be negative
sys.Parameters(3).Minimum=0;                  % Cannot be negative

compare(sys,data)
return

%% Identify sys
opt = nlgreyestOptions;
opt.Display='On' % Show progress
opt.SearchOptions.MaxIterations = 50;           % Maximal number of iterations
opt.EstimateCovariance = true;
opt.SearchMethod = 'lsqnonlin';
nidek = pem(data,sys,opt)

