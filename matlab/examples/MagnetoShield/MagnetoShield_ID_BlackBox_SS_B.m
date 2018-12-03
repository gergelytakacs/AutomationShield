%   MagnetoShield black-box linear state-space model 
%   system identification example
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
%   Last update: 3.12.2018.
  
clc; clear all;                              % Clears screen and all variables
close all;
load resultID_B
Ts=0.004                                      % [s] Sampling
u=resultID(:,1)+resultID(:,2);
y=resultID(:,3)/1000;
u0=mean(u);                                     % Input linearization
y0=mean(y);                                     % Output linearization
data = iddata(y,u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid Voltage';            % Input name
data.InputUnit =  'V';                          % Input unit
data.OutputName = 'Position';                   % Output name
data.OutputUnit = 'm';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(241:end)                             % Discard the time when magnet is on ground, pick close to linearization point
              
data = detrend(data,1)
                                                      
% State space model estimation                        
 Options = ssestOptions;                              
 Options.Display = 'on';                              
 Options.Focus = 'simulation';   
 Options.WeightingFilter = [0 460];   
 Options.EnforceStability = 0;                        
 Options.InitialState = 'estimate';                                                                                           
 Options.SearchOptions.MaxIterations=100;
 model = ssest(data, 3, Options) 
 compare(data,model)
 save MagnetoShield_Models_Blackbox_SS model
 

 
