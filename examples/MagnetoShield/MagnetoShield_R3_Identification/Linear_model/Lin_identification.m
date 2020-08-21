%% Linear model estimation

% load data
load('Data2.2.mat');
% Non-linear model parameters:
 K = 1.50848532450986e-08; % MagConstant
 m = 0.000734483128062366; % Mass
 R = 194.878001775828; % Resistance
 L = 0.102154434955465; % Inductance
 g = 10.1861174143018; % GravityOfEarth
 C = 64.9313547331575; % Damping

% Linearization points r0, v0, I0, U0
x01 = 0.0135; %r0 - height of levitation, changeable
x02 = 0;    %v0 - has to be 0 because of the stability
x03 = g*m*x01^4/K; %I0 - 1. term of linearization formula f(r0,I0,v0)
u0 = R*x03-K/(x01^4)*x02; %U0 - 1. term of linearization formula f(r0,v0,I0,U0)

% Create linear model sufficient for estimation
parameters = {'x0',x01;'v0',x02;'I0',x03;'MagConstant',K;...
    'Resistance',R; 'Inductance',L; 'Mass',m; 'Damping',C};
fcn_type = 'c'; % continous model
lin_model = idgrey('idef_lin_m.m',parameters,fcn_type);
% lin_model.StateName{1,1} = 'Position';
% lin_model.StateName{2,1} = 'Velocity';
% lin_model.StateName{3,1} = 'Current';
% lin_model.StateUnit{1,1} = 'm';
% lin_model.StateUnit{2,1} = 'm/s';
% lin_model.StateUnit{3,1} = 'A';
lin_model.InputName = 'Voltage';
lin_model.OutputName = {'Position','Current'};
lin_model.InputUnit = 'V';
lin_model.OutputUnit = {'m','A'}

% Model steady points - passed to function as parameters - are not an
% object of estimation!
lin_model.Structure.Parameters(1).Free = 0;
lin_model.Structure.Parameters(2).Free = 0;
lin_model.Structure.Parameters(3).Free = 0;

% Model parameters - are an object of estimation
lin_model.Structure.Parameters(4).Free = 1;
lin_model.Structure.Parameters(5).Free = 1;
lin_model.Structure.Parameters(6).Free = 1;
lin_model.Structure.Parameters(7).Free = 1;
lin_model.Structure.Parameters(8).Free = 1;

% Estimation options
opt = greyestOptions
opt.EnforceStability = 0;

% Measured data for estimation
% In linearized model are states no more values of r,v,I,U but their
% variation in time - need to substract steady state from measure data
input = u(1200:10000)-u0;
output = [y(1200:10000)/1000-x01, I(1200:10000)/1000-x03];
Ts = 0.001; %s
data = iddata(output,input,Ts);
set(data,'InputName',{'Voltage'});
set(data,'OutputName',{'Position','Current'});
set(data,'InputUnit',{'V'});
set(data,'OutputUnit',{'m','A'});

% Grey-box linear estimation
sys = greyest(data,lin_model,opt)

%% Model validation
% Dataset for validation
input2 = u-u0;
output2 = [y/1000-x01, I/1000-x03];
Ts = 0.001; %s
data2 = iddata(output2,input2,Ts);
set(data2,'InputName',{'Voltage'});
set(data2,'OutputName',{'Position','Current'});
set(data2,'InputUnit',{'V'});
set(data2,'OutputUnit',{'m','A'});

compare(data2,sys)
%[model_outputs,~,~]=compare(data2,sys)