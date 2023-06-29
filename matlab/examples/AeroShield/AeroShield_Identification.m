%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%   Created by Jan Boldocky.
%   Last update: 16.1.2023.
clear
close all
clc

load Identification_Data.mat
Ts = 0.015;
data = iddata(yrad, u, Ts, 'Name', 'Experiment');
data = detrend(data);
%data = data(800:end);
data.InputName = 'Voltage';
data.InputUnit = 'V';
data.OutputName = 'Pendulum Angle';
data.OutputUnit = 'rad';
data.Tstart = 0;
data.TimeUnit = 's';

load Validation_Data.mat
Ts = 0.015;
v_data = iddata(yradval, uval, Ts, 'Name', 'Experiment');
[v_data, trendData] = detrend(v_data);

theta0 = data.y(1,1);
dtheta0 = (data.y(2,1) - data.y(1,1))/Ts;
%% Initial guess of parameters
m = 0.005;
g = 9.81;
l = 0.1;
c = 0.001;
K = 0.01;
InitialStates = [theta0; dtheta0];
Ts = 0;
modelType = 'nonlinear';
%modelType = 'linear';

switch modelType
    case 'nonlinear'
% Nonlinear greybox model identification
FileName = 'AeroShield_ODE';
Order = [1, 1, 2];
Parameters = {m, g, l, c, K};

nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, 'Name', 'Nonlinear Grey-box Model');
set(nlgr, 'InputName', 'Voltage', 'InputUnit', 'V', 'OutputName', 'Pendulum Angle', 'OutputUnit', 'rad', 'TimeUnit', 's');
nlgr = setpar(nlgr, 'Name', {'Pendulum Mass', 'Gravitational Acceleration', 'Pendulum Length', 'Viscous Damping', 'Thrust'});
nlgr = setinit(nlgr, 'Name',{'Pendulum Angle', 'Pendulum Angular Velocity'});

nlgr.Parameters(1).Fixed = true;
nlgr.Parameters(1).Minimum = 0.005;
nlgr.Parameters(1).Maximum = inf;

nlgr.Parameters(2).Fixed = true;
nlgr.Parameters(2).Minimum = 0;
nlgr.Parameters(2).Maximum = 10.0;

% nlgr.Parameters(3).Fixed = false;
% nlgr.Parameters(3).Minimum = 0.001;
% nlgr.Parameters(3).Maximum = inf;
% 
% nlgr.Parameters(4).Fixed = false;
% nlgr.Parameters(4).Minimum = 0;
% nlgr.Parameters(4).Maximum = inf;
% 
% nlgr.Parameters(5).Fixed = false;
% nlgr.Parameters(5).Minimum = 0;
% nlgr.Parameters(5).Maximum = inf;

nlgr.InitialStates(1).Fixed = false;
nlgr.InitialStates(2).Fixed = false;
nlgr.SimulationOptions.Solver = 'ode1';

size(nlgr)
present(nlgr)
opt = nlgreyestOptions('Display', 'on', 'EstCovar', true, 'SearchMethod', 'Auto');
opt.SearchOption.MaxIter = 40;
model = nlgreyest(data, nlgr, opt);

%% Linearization of the identified model
m_ = model.Parameters(1).Value;
g_ = model.Parameters(2).Value;
l_ = model.Parameters(3).Value;
c_ = model.Parameters(4).Value;
K_ = model.Parameters(5).Value;
syms dxdt y x1 x2 u
dxdt=[x2;
    u*(K_/(l_*m_))-(g_/l_)*sin(x1)-x2*(c_/(m_*l_^2))];
y=[x1;
    x2];

Alin=jacobian(dxdt,[x1 x2]);
Blin=jacobian(dxdt,u);
Clin=jacobian(y,[x1 x2]);
Dlin=jacobian(y,u);
xlin=[pi/4 0];
ulin=0;
Ac=eval(subs(Alin,[x1 x2 u],[xlin ulin]));
Bc=eval(subs(Blin,[x1 x2 u],[xlin ulin]));
Cc=eval(subs(Clin,[x1 x2],xlin));
Dc=eval(subs(Dlin,[x1 x2],xlin));
linsys=ss(Ac,Bc,Cc,Dc);
save AeroShield_GreyboxModel_Nonlinear.mat model
save AeroShield_GreyboxModel_Linear.mat linsys

%% Model Validation
compare(v_data,model,linsys)

    case 'linear'
A = [0 1;
    -g/l -c/(m*l^2)];
B = [0;
    K/(l*m)];

C = [1 0];

D = [0];
K = zeros(2,1);

disp('Initial guess:')
sys = idss(A,B,C,0,K,InitialStates,0)
sys.Structure.A.Free = [0 0;
                        1 1];
sys.Structure.B.Free = [0;
                        1];
sys.Structure.C.Free = false;
sys.DisturbanceModel = 'estimate';
%sys.InitialState = 'estimate';

Options = ssestOptions;
Options.Display = 'on';                    
Options.Focus = 'simulation';              % Focus on simulation
%Options.SearchMethod = 'grad'
Options.SearchOptions.MaxIterations = 40;  % Max iterations
Options.InitialState = 'estimate';         % Estimate initial condition
disp('Estimated model:')                       
linsys = ssest(data,sys,Options);           % Identified Model

lest = -g/linsys.A(2,1) %Calculation of parameters
cest = -linsys.A(2,2)*lest*m
Kest = linsys.B(2,1)*lest*m
compare(data,model)
end