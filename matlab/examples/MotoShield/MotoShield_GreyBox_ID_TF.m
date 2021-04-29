%   MotoShield grey-box identification procedure
% 
%   Identification of a grey-box model using experimental data.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács and Ján Boldocký.

clc; clear; close all;

%% Data preprocessing
load aprbsResult.mat                            %_Load data
Ts = 0.02;                                      %_Sampling Period
data = iddata(y,u,Ts,'Name','Experiment');      %_Identification data object
data = detrend(data);                           % Remove offset and linear trends  
data.InputName = 'Input Voltage';               %_Input
data.InputUnit =  'V';                          %_Input unit
data.OutputName = 'Angular Velocity';           %_Output
data.OutputUnit = 'rad/s';                      %_Output unit
data.Tstart = 0;                                %_Time offset
data.TimeUnit = 's';                            %_Time unit
%% Initial guess of model parameters
J = 0.01;       %_Moment of inertia
b = 0.01;        %_Viscous friction constant
Ke = 1;     %_Electromotive force constant # Voltage over Velocity
Kt = 1;     %_Motor torque constant # Torque over Current
R = 10.3;         %_Electric resistance
L = 0.1;        %_Electric inductance

b0 = Kt;                         %_Gain # Numerator
a2 =  J*L;                       %_Polynomial coefficients # Denominator
a1 = (J*R+b*L);
a0 = (b*R+Kt^2);

disp('Initial model:')
sys = idtf([b0],[a2 a1 a0])                   %_Initial guess of the model
sys.Structure.Denominator.Maximum = [1000 1000 inf];
sys.Structure.Denominator.Minimum = [0 0 0];
sys.Structure.Numerator.Maximum= inf;
sys.Structure.Numerator.Minimum= 0;
Options = tfestOptions;                        %_Options # tfest
Options.Display = 'on';                        
Options.InitialCondition = 'estimate';         
Options.SearchOptions.MaxIterations = 100; %_Maximum number of iterations

%_Identification
disp('Estimated model:')                       
model = tfest(data,sys,Options)                %_Identified model
zmodel = c2d(model,Ts);                        %_Discrete model
compare(data,model);                           %_Model/data comparison
                                               
grid on                                       
save MotoShield_GreyboxModel_TF model zmodel
