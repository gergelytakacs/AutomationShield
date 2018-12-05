%   MagnetoShield black-box linear state-space model 
%   system identification example
% 
%   This example takes a measurement from the 
%   MagnetoShield device, where the permanent magnet is
%   levitatated around the 15 mm setpoint in closed-loop
%   using PID feedback. The identification assumes that 
%   the dynamics can be represented using a differential
%   model (from this equilibrium) expressed by the 
%   classical mechanical equation, where current is
%   related to voltage linearly, e.g. 
%   mdy^2/dt^2=mg-K*Ki*u^2/y^2,
%   where y is the distance from the electromagnet,
%   i=Ki*u is the linear equivalent current, K is the
%   magnetic (soleonoid) constant. After linearization
%   This becomes a second order TF, modeled below.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 5.12.2018.
  
clc; clear all;                                 % Clears screen and all variables
close all;                                      % Closes all figures
load resultID.mat;                              % Loads data file
Ts=0.004;                                       % [s] Sampling
u=resultID(:,1)+resultID(:,2);                  % [V] Input and probe signal
y=resultID(:,3)/1000;                           % [m] Output (position)

data = iddata(y,u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid Voltage';            % Input name
data.InputUnit =  'V';                          % Input unit
data.OutputName = 'Position';                   % Output name
data.OutputUnit = 'm';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(241:end);                           % Discard the time when magnet is on ground, pick close to linearization point              
data = detrend(data,1);                         % Remove steady-state component to get delta formulation
dataf = fft(data);                              % Frequency domain (tfest() handles unstable models only in f-domain)

%% Parameters
m = 0.76E-3;                                    % [kg] Magnet mass
u0=mean(u);                                     % [V] Input linearization around setpoint
y0=mean(y);                                     % [m] Output linearization around setpoint
K=0.5E-7;                                       % Magnetic constant * voltage to current constant

%% Linearization
% The more Voltage, the closer to magnet, therefore b0
% must be negative. From the linearization it also
% follows that a0 is negative.
a0=-2*(K/m)*u0^2/y0^3
b0=-2*(K/m)*u0/y0^2


%% Creating model structure
sys = idtf([b0],[1 0 a0]);
sys.Structure.Denominator.Free = [0 0 1];
sys.Structure.Denominator.Minimum=[-Inf -Inf -Inf]; 
sys.Structure.Denominator.Maximum=[Inf 0 0];

%% Configure estimation procedure
Options = tfestOptions;                         % Identification options for ssest
Options.Display = 'on';                         % Show progress
Options.EnforceStability = 0;                   % Unstable models allowed
Options.EstimateCovariance = 1;                 % Estimate parameter covariance
Options.SearchMethod='auto';                    % Search method gn, gna, lm, grad, lsqnonlin, fmincon
Options.SearchOptions.MaxIterations=10;         % Set maximal iterations%
Options.InitializeMethod = 'all';               % Try all available initialization methods
Options.InitialCondition = 'estimate';          % Estimate initial condition as well

%% Identify model
model=tfest(sys,dataf,Options)                  % Grey box TF

%% Compare to data spectra
compare(model, dataf)

%% Step response in closed-loop
Kp=3195; Ki=16154; Kd=50.9621;                  % Test tuning
P=model;                                        % Plant
C=tf([Kd Kp Ki],[-1 0]);                        % Controller
S=feedback(C*P,1);                              % Closed-loop TF
step(S)                                         % Step response

%% Save model
save MagnetoShield_Models_Greybox_TF model     % Save model


