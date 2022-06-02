%   MotoShield system Identification example
% 
%   This is an example of MotoShield transfer function identification.
%   At the end of the script a step-response tangent is drawn
%   
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács and Ján Boldocký.

startScript;

%% Data preprocessing
load identification_data.mat
Ts = 0.02;                                      %_Sampling Period
y=y/Ts*2*pi/28;             %_PPS to rad/sec
trenddata = iddata(y(1:end),u(1:end),Ts,'Name','Experiment');      %_Identification data object
%data = detrend(trenddata);                           % Remove offset and linear trends  
data=trenddata(500:2500)
data.InputName = 'Input Voltage';               %_Input
data.InputUnit =  'V';                          %_Input unit
data.OutputName = 'Angular Velocity';           %_Output
data.OutputUnit = 'rad/s';                      %_Output unit
data.Tstart = 0;                                %_Time offset
data.TimeUnit = 's';                            %_Time unit
%% Initial guess of model parameters
J = 1e-5;       %_Moment of inertia
b = 1e-5;        %_Viscous friction constant
Ke = 1e-2;     %_Electromotive force constant # Voltage over Velocity
Kt = 1e-2;     %_Motor torque constant # Torque over Current
R = 10;         %_Electric resistance # Measured Value
L = 3500e-6;        %_Electric inductance # Measured Value

b0 = Kt;                         %_Gain # Numerator
a2 =  J*L;                       %_Polynomial coefficients # Denominator
a1 = (J*R+b*L);
a0 = (b*R+Kt^2);

disp('Initial model:')
sys = idtf([b0],[a2 a1 a0])                   %_Initial guess of the model
sys.Structure.Denominator.Maximum = [inf inf inf];
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
zmodel = c2d(model,0.02);                        %_Discrete model
compare(data,model);                           %_Model/data comparison
                                               
grid on                                       
save MotoShield_GreyboxModel_TF model zmodel
%% Step Response Tangent
opt = stepDataOptions('InputOffset',0,'StepAmplitude',5);
[y,t] = step(zmodel,opt)
dy=diff(y,1)
dydt=diff(y)./diff(t)
[dy_max,dy_max_i]=max(dy)
inflex_y=y(dy_max_i)
inflex_t=t(dy_max_i)
step(model,opt)
hold on 
scatter(inflex_t,inflex_y)
tangent=(t-t(dy_max_i))*dydt(dy_max_i)+y(dy_max_i)
plot(t,tangent)
line=polyfit(t,tangent,1)
tangent_time= @(y) (y-line(2))/line()