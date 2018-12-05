%   MagnetoShield black-box linear state-space model 
%   system identification example
% 
%   This example takes a measurement from the 
%   MagnetoShield device, where the permanent magnet is
%   levitatated around the 15 mm setpoint in closed-loop
%   using PID feedback. The identification assumes that 
%   the dynamics can be represented using a differential
%   model (from this equilibrium) contained within a 
%   first order transfer function. No assumptions are
%   made on the TF, this is a black-box identfification 
%   procedure. 
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
u0=mean(u);                                     % Input linearization around setpoint
y0=mean(y);                                     % Output linearization around setpoint
data = iddata(y,u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid Voltage';            % Input name
data.InputUnit =  'V';                          % Input unit
data.OutputName = 'Position';                   % Output name
data.OutputUnit = 'm';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(241:end);                           % Discard the time when magnet is on ground, pick close to linearization point              
data = detrend(data,1);                         % Remove steady-state component to get delta formulation

% Estimate
Options = tfestOptions;                         % Identification options for ssest
Options.Display = 'on';                         % Show progress
Options.EnforceStability = 0;                   % Unstable models allowed
Options.EstimateCovariance = 1;                 % Estimate parameter covariance
Options.WeightingFilter = [0 400];              % [rad] Pre-filter data 
Options.SearchOptions.MaxIterations=200;        % Set maximal iterations%model=pem(sys,data)
Options.InitializeMethod = 'all';               % Try all available initialization methods
Options.InitialCondition = 'estimate';          % Estimate initial condition as well
model = tfest(data, 1, 0, Options)              % Black box TF

%% Simulation comparison
t=0:Ts:(length(data.u)-1)*Ts;                   % Time vector
[Y,T,X]=lsim(model,data.u,t);                   % Linear simulation
plot(T,Y);                                      % Plot model response
hold on                                         % Hold on plots
plot(T,data.y);                                 % Plot original data
xlabel('Time (s)')                              % X-axis label
ylabel('Magnet position (m)')                   % Y-axis label
legend('Model','Data')                          % Legend
grid on                                         % Turn on grid


%% Save model
save MagnetoShield_Models_BlackBox_TF model    % Save model