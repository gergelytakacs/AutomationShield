clc;
clear;
%% Non-linear model estimation

load('IDExperiment1');
y=result(:,1);
u=result(:,2);
I=result(:,3);

% Parameters initinal value
 K =    1e-08; % MagConstant
 m =    0.78 / 1000;                      % [kg] Mass, measured
 R =    198.0;                            % [Ohm] Resistance, measured
 L =    0.1; % Inductance                 % [H]
 g =    9.81; % GravityOfEarth            % [ms^-2] Constant
 C =    50; % Damping

% Dataset for estimation
start  =  1;
stop   =  length(u);
U      =   u(start:stop); % voltage
Y      =  [y(start:stop)/1000, I(start:stop)/1000]; % position[m] ; current[A]
Ts     =  0.003250; %s
data   =  iddata(Y,U,Ts);


% Create model sufficient for non-lin estimation
Order         = [2 1 3]; % Model order [ny nu nx].
Parameters    = [K; m; R; L; g];    % Initial parameter vector.
InitialStates = [Y(1,1); 0 ; Y(1,2)];  % Initial values of initial states.
nlModel       = idnlgrey('MagnetoShield_ODE', Order, Parameters, InitialStates,0)

%% Setting up paa
nlModel.InputName          = 'Voltage';
nlModel.InputUnit          = 'V';
nlModel.OutputName{1,1}    = 'Position';
nlModel.OutputName{2,1}    = 'Current';
nlModel.OutputUnit{1,1}    = 'm';
nlModel.OutputUnit{2,1}    = 'A';

% Parameter names
nlModel.Parameters(1).Name = 'MagConstant';
nlModel.Parameters(2).Name = 'Mass';
nlModel.Parameters(3).Name = 'Resistance';
nlModel.Parameters(4).Name = 'Inductance';
nlModel.Parameters(5).Name = 'GravityOfEarth';

% Parameter units
nlModel.Parameters(1).Unit = 'N/A';
nlModel.Parameters(2).Unit = 'kg';
nlModel.Parameters(3).Unit = 'Ohm';
nlModel.Parameters(4).Unit = 'H';
nlModel.Parameters(5).Unit = 'm/s^2';


% Which params to estimate
nlModel.Parameters(1).Fixed = 0; % K
nlModel.Parameters(2).Fixed = 0; % m_h
nlModel.Parameters(3).Fixed = 0; % R
nlModel.Parameters(4).Fixed = 0; % L
nlModel.Parameters(5).Fixed = 0; % g

% %Estimation constraints
% nlModel.Parameters(1).Minimum = 1E-9;%K
% nlModel.Parameters(1).Maximum = 1E-7;
% nlModel.Parameters(2).Minimum = m/1.5;    % 10% change allowed
% nlModel.Parameters(2).Maximum = m*1.5;    % 10% change allowed
% nlModel.Parameters(3).Minimum = R/1.5;    % 10% change allowed
% nlModel.Parameters(3).Maximum = R*1.5;    % 10% change allowed
% nlModel.Parameters(4).Minimum = L/1.5;    % 50% change allowed
% nlModel.Parameters(4).Maximum = L*1.5;    % 50% change allowed
% nlModel.Parameters(5).Minimum = g/1.007;  % 0.7% max change on Earth = g*1.007;  %0.7% max change on Earth
% nlModel.Parameters(5).Maximum = g*1.007;  % 0.7% max change on Earth = g*1.007;  %0.7% max change on Earth


nlModel.InitialStates(1).Fixed   = false; 
nlModel.InitialStates(2).Fixed   = false; 
nlModel.InitialStates(3).Fixed   = false; 

% Estimation options
opt = nlgreyestOptions;
opt.Display = 'on';
opt.OutputWeight=[100 0;0 1];
%opt.SearchMethod= 'lsqnonlin';
opt.EstCovar = true;
opt.SearchOptions.MaxIterations = 30;

% Non-linear estimation by grey-box model
sys = nlgreyest(data,nlModel,opt);

%% Validation of the model
input2 = u;
output2 = [y/1000, I/1000];
Ts = 0.001; %s
data2 = iddata(output2,input2,Ts);
set(data2,'InputName',{'Voltage'});
set(data2,'OutputName',{'Position','Current'});
set(data2,'InputUnit',{'V'});
set(data2,'OutputUnit',{'m','A'});

compare(data2,sys)
%[model_outputs,~,~]=compare(data2,sys) 