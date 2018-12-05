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

mu0 = 4*pi*1E-7;                               % [H/m] Permeability of free space
mur = 10;                                      % [-] Ratio for the permeability of the core vs. free space 
mu = mu0*mur;                                  % [H/m] Permeability of the core
N =  2500;                                     % [-] Number of windings (turns)
d = 8E-3;                                      % [m] Solenoid core diameter
A = pi*(d/2)^2;                                % [m^2] Solenoid core cross-sectional area
K=(mu^2*N^2*A);                                % Solenoid (magnetic) constant

% The more Voltage, the closer to magnet, therefore b0
% must be negative. From the linearization it also
% follows that a0 is negative.
a0=-2*(K/m)*u0^2/y0^3
b0=-2*(K/m)*u0/y0^2

% Creating model structure
sys = idtf([b0],[1 0 a0]);
sys.Structure.Denominator.Free = [0 1 1];
sys.Structure.Denominator.Minimum=[-Inf -Inf -Inf]; 
sys.Structure.Denominator.Maximum=[Inf 0 0];

% Configure estimation procedure
Options = tfestOptions;                         % Identification options for ssest
Options.Display = 'on';                         % Show progress
Options.EnforceStability = 0;                   % Unstable models allowed
Options.EstimateCovariance = 1;                 % Estimate parameter covariance
Options.SearchOptions.MaxIterations=100;        % Set maximal iterations%model=pem(sys,data)
Options.InitializeMethod = 'all';               % Try all available initialization methods
Options.InitialCondition = 'estimate';          % Estimate initial condition as well


model=tfest(sys,dataf,Options)                  % Grey box TF
compare(model, dataf)
compare(model,data)

return

%%
% fact=0.01;
% datas=idresamp(data,fact)
% modeld=c2d(model,fact*Ts);
% compare(modeld, datas)
% axis([0 15,-1.5E-3,1.5E-3])

Kp=-2821;
Ki=-11556;
Kd=-46;
P=model;                    % Plant
C=tf([Kd Kp Ki],[1 0]);     % Controller
S=feedback(C*P,1);          % Closed-loop TF
step(S)


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
save MagnetoShield_Models_Greybox_TF model    % Save model


