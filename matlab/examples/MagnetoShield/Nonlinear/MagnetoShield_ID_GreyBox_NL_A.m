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
  
%   MagnetoShield grey-box linearized state space system
%   identification example
% 
%   This example takes a measurement from the 
%   MagnetoShield device, where the permanent magnet is
%   levitatated around the 15 mm setpoint in closed-loop
%   using PID feedback. The identification assumes that 
%   the dynamics can be represented using third order
%   state-space model, that is based on the second order
%   mechanical equation and the first order electrical
%   equation. Most literature asssumes that the 
%   inductance L of the solenoid is fixed 
%   (fixedInductance=1), here you also may choose to
%   model it as a distance-dependent variable 
%   (fixedInductance=0). The resulting model will be in
%   "delta" form, that is all variables relate to 
%   linearization point.

%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 7.12.2018.
  
clc; clear all;                                 % Clears screen and all variables
load resultID.mat                               % Load data file
Ts=0.004                                        % [s] Sampling
u=resultID(:,1)+resultID(:,2);                  % [V] Input is closed loop + probe signal
y=resultID(:,3)/1000;                           % [m] Output in meters

%% System identification data object
data = iddata(y,u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid Voltage';            % Input name
data.InputUnit =  'V';                          % Input unit
data.OutputName = 'Position';                   % Output name
data.OutputUnit = 'm';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(241:end)                            % Discard the time when magnet is on ground, pick close to linearization point
data=detrend(data);

%% Parameters
m = 0.76E-3;                                    % [kg] Magnet mass
R = 198.3;                                      % [Ohm] Solenoid resistance
L = 0.239;                                      % [H] Solenoid inductance
K=4.5E-6;                                       % Magnetic constant (rhough estimate)
    
% Initial parameters for linearized model
h0=data(1).y;                                   % Initial position estimate
dh0=(data(2).y-data(1).y)/Ts;                   % Initial velocity estimate
i0= data(1).u/R;                                % Initial current estimate

FileName      = 'MagnetoShield_ODE';            % File describing the model structure.
Order         = [1 1 3];                        % Model orders [ny nu nx].
Parameters    = {K,R,L};             % Initial parameters.
InitialStates = [h0; dh0; i0];                  % Initial state.
sys = idnlgrey(FileName, Order, Parameters, InitialStates, 0, ...
                'Name', 'Magnetic Levitation');
sys = setinit(sys, 'Fixed', true);              % Estimate the initial states.
%model.Parameters(1).Minimum=0;                 % Cannot be negative
model.Parameters(2).Minimum=0;                  % Cannot be negative
model.Parameters(2).Maximum=1000;               % Most likely around 200 Ohm
model.Parameters(3).Minimum=0;                  % Cannot be negative
model.Parameters(3).Maximum=2;                  % Most likely around 0.5 H

%% Identify model
opt = nlgreyestOptions;
opt.Display='On' % Show progress
opt.SearchMethod = 'fmincon'; %lm, gn, gna, lsqnonlin, fmincon
opt.SearchOptions.MaxIterations = 50;           % Maximal number of iterations
%opt.Searchoptions.Tolerance=1E-8;
%opt.Searchoptions.StepTolerance=1E-8; %1E-6 default for fmincon
%opt.Searchoptions.FunctionTolerance=1E-8; %1E-6 default for fmincon
opt.EstimateCovariance = true;
%opt.Advanced.ErrorThreshold = 0;
opt.Regularization.Nominal='model';

model=nlgreyest(data,sys,opt)
%model2 = pem(dataf,model,opt)
model.Parameters(1).Value
model.Parameters(2).Value
model.Parameters(3).Value
