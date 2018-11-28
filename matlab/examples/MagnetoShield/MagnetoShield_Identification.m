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
y=resultID(:,3);

data = iddata(y,u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid Voltage';                  % Input name
data.InputUnit =  'V';                      % Input unit
data.OutputName = 'Position';                 % Output name
data.OutputUnit = 'mm';                        % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
 
                                    
 % Import   mydata                  
datad = detrend(data,0)        
model_init = idtf([80],[1 0 620]);
model_init.Structure.Denominator.Free=[0 0 1];
% model_init.Numerator.Minimum = eps;
% model_init.Structure.Denominator.Minimum = eps;
% model_init.Structure.Denominator.Free(end) = false;
opt = tfestOptions('SearchMethod','lsqnonlin'); %'gn', 'gna', 'lm', 'grad','lsqnonlin' or 'fmincon'.
model = tfest(datad,model_init,opt);
model
compare(datad,model)
return

 %% Working black box
% Transfer function estimation      
 Options = tfestOptions;            
 Options.Display = 'on';            
 Options.WeightingFilter = [];      
                                    
 tf1 = tfest(mydatad, 2, 1, Options)