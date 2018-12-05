%   MagnetoShield black-box linear state-space model 
%   system identification example
% 
%   This example takes a measurement from the 
%   MagnetoShield device, where the permanent magnet is
%   levitatated around the 15 mm setpoint in closed-loop
%   using PID feedback. The identification assumes that 
%   the dynamics can be represented using a differential
%   model (from this equilibrium) contained within a 
%   third order state-space system. No assumptions are
%   made about the state-space system, thus this is a
%   black-box identfification procedure. The state has
%   no physcally valid meaning.
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
dataf = fft(data);                              % Frequency domain (tfest() handles unstable models only in f-domain)
% Parameters
m = 0.76E-3;                                    % [kg] Magnet mass
R = 198.3;                                      % [Ohm] Solenoid resistance
L = 0.239;                                      % [H] Solenoid inductance

%mu0 = 4*pi*1E-7;                               % [H/m] Permeability of free space
%mur = 4000                                     % [-] Ratio for the permeability of the core vs. free space (~2000-6000)
%mu = mu0*mur;                                  % [H/m] Permeability of the core
%N =  250;                                      % [-] Number of windings (turns)
%d = 8E-3;                                      % [m] Solenoid core diameter
%A = pi*(d/2)^2;                                % [m^2] Solenoid core cross-sectional area
%K=(mu^2*N^2*A)/(2*mu0);                        % Solenoid constant

h0=data(1).y;                                   % Initial position estimate
dh0=(data(2).y-data(1).y)/Ts;                   % Initial velocity estimate
i0=data(1).u/R;                                 % Initial current estimate
                                                      
% Configure estimation procedure

a0=4000;
b0=0.1;

sys = idtf([b0],[1 0 a0]);
sys.Structure.Denominator.Free = [0 0 1];
sys.Structure.Denominator.Minimum=[-Inf -Inf -Inf];
sys.Structure.Denominator.Maximum=[Inf Inf Inf];

Options = tfestOptions;                         % Identification options for ssest
Options.Display = 'on';                         % Show progress
Options.EnforceStability = 0;                   % Unstable models allowed
Options.EstimateCovariance = 1;                 % Estimate parameter covariance
Options.WeightingFilter = [0 400];              % [rad] Pre-filter data 
Options.SearchOptions.MaxIterations=200;        % Set maximal iterations%model=pem(sys,data)
Options.InitializeMethod = 'all';               % Try all available initialization methods
Options.InitialCondition = 'estimate';          % Estimate initial condition as well


model=tfest(sys,dataf,Options)                  % Grey box TF
compare(model, dataf)
return


model = tfest(data, 2, 0, Options);             % Black box TF

%% Simulation comparison
t=0:Ts:(length(data.u)-1)*Ts;
[Y,T,X]=lsim(model,data.u,t);
plot(T,Y);
hold on
plot(T,data.y);
xlabel('Time (s)')
ylabel('Magnet position (m)')

%% Save model
save MagnetoShield_Models_Greybox_TF model    % Save model



save MagnetoShield_Models_Blackbox_SS model     % Save model