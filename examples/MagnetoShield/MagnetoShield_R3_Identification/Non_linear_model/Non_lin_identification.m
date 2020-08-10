%% Non-linear model estimation

load('Data2.0.mat');
% Prameters initinal value
 K = 1.50848532450986e-08; % MagConstant
 m_h = 0.000734483128062366; % Mass
 R = 194.878001775828; % Resistance
 L = 0.102154434955465; % Inductance
 g = 10.1861174143018; % GravityOfEarth
 C = 64.9313547331575; % Damping

% Dataset for estimation
input = u(1200:10000); % voltage
output = [y(1200:10000)/1000, I(1200:10000)/1000]; % position[m] ; current[A]
Ts = 0.001; %s
data = iddata(output,input,Ts);

% Create model sufficient for non-lin estimation
Order = [2 1 3]; % Model order [ny nu nx].
Parameters    = [K; m_h; R; L; g; C];   % Initial parameter vector.
InitialStates = [output(1,1); 0; output(1,2)]; % Initial values of initial states.
nlModel    = idnlgrey('identified_nonlin_model_m', Order, Parameters,...
    InitialStates,0)
nlModel.InputName = 'Voltage';
nlModel.InputUnit = 'V';
nlModel.OutputName{1,1} = 'Position';
nlModel.OutputName{2,1} = 'Current';
nlModel.OutputUnit{1,1} = 'm';
nlModel.OutputUnit{2,1} = 'A';

% Parameters names
nlModel.Parameters(1).Name = 'MagConstant';
nlModel.Parameters(2).Name = 'Mass';
nlModel.Parameters(3).Name = 'Resistance';
nlModel.Parameters(4).Name = 'Inductance';
nlModel.Parameters(5).Name = 'GravityOfEarth';
nlModel.Parameters(6).Name = 'Damping';

% Parameters units
nlModel.Parameters(1).Unit = 'N/A';
nlModel.Parameters(2).Unit = 'kg';
nlModel.Parameters(3).Unit = 'Ohm';
nlModel.Parameters(4).Unit = 'H';
nlModel.Parameters(5).Unit = 'm/s^2';
nlModel.Parameters(6).Unit = '1/s';

% Which params to estimate
nlModel.Parameters(1).Fixed = 0; % K
nlModel.Parameters(2).Fixed = 0; % m_h
nlModel.Parameters(3).Fixed = 0; % R
nlModel.Parameters(4).Fixed = 0; % L
nlModel.Parameters(5).Fixed = 0; % g
nlModel.Parameters(6).Fixed = 0; % C

%Estimation interval
nlModel.Parameters(1).Minimum = [0.93928625653008e-09];%K
nlModel.Parameters(1).Maximum = [9.43928625653008e-08];
nlModel.Parameters(2).Minimum = 0.0005;%m_h
nlModel.Parameters(2).Maximum = 0.0009;
nlModel.Parameters(3).Minimum = 170;%R
nlModel.Parameters(3).Maximum = 220;
nlModel.Parameters(4).Minimum = 0.09;%L
nlModel.Parameters(4).Maximum = 0.32;
nlModel.Parameters(5).Minimum = 9.4;%g
nlModel.Parameters(5).Maximum = 10.2;
nlModel.Parameters(6).Minimum = -inf;%C
nlModel.Parameters(6).Maximum = inf;

% Estimation options
opt = nlgreyestOptions;
opt.Display = 'on';
opt.OutputWeight=[100 0;0 1];
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