%   MagnetoShield grey-box linear transfer function
%   model system identification example
% 
%   This example takes a measurement from the 
%   MagnetoShield device, where the permanent magnet is
%   levitatated around the 15 mm setpoint in closed-loop
%   using PID feedback. The identification assumes that 
%   the dynamics can be represented using a differential
%   model (relative to this equilibrium) expressed by 
%   the classical mechanical equation for the position 
%   of the disc, where the incoming current is related 
%   to voltage accross the electromagnet by Kirchoff's 
%   law. After linearization this becomes a third order 
%   TF, as modeled below.
%   
%   As the target model is open-loop unstable, we have
%   to perform the identification task in the frequency
%   domain. Note, that a time-domain simulation and 
%   verification can be only performed in closed-loop.
%   Though the fit of the model to data is ~83%, the 
%   time domain simulation reveals the inaccuracies of 
%   the linear model
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 7.9.2020.
  
clc; clear all;                                 % Clears screen and all variables
close all;                                      % Closes all figures
load MagnetoShield_ID_Data.mat                          % Load data file
Ts=0.00325;                                       % [s] Sampling
y=result(:,1)/1000;                             % [m] Output (position)
u=result(:,2);                                  % [V] Input and probe signal
i=result(:,3)/1000;                             % [i] Current
                                                
data = iddata(y,u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid Voltage';            % Input name
data.InputUnit =  'V';                          % Input unit
data.OutputName = 'Position';                   % Output name
data.OutputUnit = 'm';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(100:end);                           % Discard the time when magnet is on ground, pick close to linearization point              
data = detrend(data,1);                         % Remove steady-state component to get delta formulation
dataf = fft(data);                              % Frequency domain (tfest() handles unstable models only in f-domain)

%% Parameters
m = 0.76E-3;                                    % [kg] Magnet mass
R = 198.3;                                      % [Ohm] Solenoid resistance
L = 0.239;                                      % [H] Solenoid inductance
u0=mean(u);                                     % [V] Input linearization around setpoint
y0=mean(y);                                     % [m] Output linearization around setpoint
i0=mean(i);                                     % [A] Current linearization point
K=5E-3;                                         % Magnetic constant 
                                                % Initial guess
disp(['Linearized around ',num2str(y0*1000),' mm, at ',num2str(u0),' V.'])

%% Creating model structure
b0= -(2*K*i0)/(m*L*y0^2);                       % Gain
a2=  (R/L);                                     % Polynomial coefficients
a1=  ((2*K*i0^2)*(2*K-L*y0))/(m*L*y0^4)
a0= -(2*K*R*i0^2)/(m*L*y0^3);

disp('Initial model:')
sys = idtf([b0],[1 a2 a1 a0])                   % Initial guess of the model

%% Configure estimation procedure
Options = tfestOptions;                         % Identification options for ssest
Options.Display = 'on';                         % Show progress
Options.EnforceStability = 0;                   % Unstable models allowed
Options.InitialCondition = 'estimate';          % Estimate initial condition as well

%% Identify model
disp('Estimated model:')                        % Identified model
model=tfest(sys,dataf,Options)                  % Grey box TF

%% Compare to data spectra
figure(1)                                       % New figure
compare(model, dataf);                          % Compare to original spectra
model=model*1000;                               % Plant model, but in mm for better scaling
save MagnetoShield_Models_Greybox_TF model      % Save plant model (mm)
