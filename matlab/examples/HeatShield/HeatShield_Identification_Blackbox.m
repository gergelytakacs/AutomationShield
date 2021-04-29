%   HeatShield black-box identification procedure
% 
%   Identification of a black-box model for the HeatShield 
%   based on experimental data.
%
%   This example takes measurement data from the HeatShield, then
%   uses attempts to identify a simple first order process model.
%   Due to the presence of the constant term in the differential
%   equation, though is a linear equation it is not a linear
%   map (non-homogeneous and additive). A black-box nonlinear
%   identification yields better results.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 19.10.2018.                            

clc; clear;
% result=readExperiment(8100,'COM22',9600);
% save resultID result

load resultID2                                  % Load measurement data

%% Create system identification data object
Ts=2;                                           % [s] Sampling
Y=resultID(:,1);                                % [oC] Output
U=resultID(:,2);                                % Duty cycle of input
data = iddata(Y,U,Ts,'Name','Printer Head');    % Data file
data.InputName = 'Duty cycle';                  % Input name
data.InputUnit =  'Tp/Td';                      % Input unit
data.OutputName = 'Temperature';                % Output name
data.OutputUnit = '^oC';                        % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit

%% Process model
Opt = procestOptions;                           % Process model 
model_process = procest(data,'P1', Opt);        % Identify process model

%% Nonlinear model                                    
model_nlarx= nlarx(data,[1 1 0],'linear');      % Identify nonlinear arx

%% Compare data and save models
compare(data, model_process, model_nlarx)       % Compare data

save HeatShield_Models_Blackbox.mat model_process model_nlarx