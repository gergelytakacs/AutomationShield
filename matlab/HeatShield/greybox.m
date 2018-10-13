% result=readExperiment(8100,'COM22',9600);
% save resultID result

clc; clear;

load resultID

Ts=2;
Tabs=273.15;
Troom=25;
Y=resultID(:,1)+Tabs;
U=resultID(:,2)/20;

data = iddata(Y,U,Ts,'Name','Printer Head');      
data.InputName = 'Voltage';
data.InputUnit =  'V';
data.OutputName = 'Temperature';
data.OutputUnit = 'K';
data.Tstart = 0;
data.TimeUnit = 's';

%% Grey box nonlinear????

rho = 2700;             % Density of aluminum
m = 20*12*16*1E-9*rho;  % Weight of the block
c = 0.900;              % Specific heat of aluminum
R = 18.6;               % (Ohm)
h = 200;                % (W/(m2K)) Convective heat transfer coefficient
A = 1504E-6;            % (m2)
alpha         = 1/(m*c*R);
beta          = 1/((h*A)*(m*c));

alpha         = 500;
beta          = 35000;

FileName      = 'printerheat';     % File describing the model structure.
Order         = [1 1 1];           % Model orders [ny nu nx].
Parameters    = [alpha; beta];         % Initial parameters. Np = 2.
InitialStates = [Troom];              % Initial initial states.
Ts            = 0;                 % Time-continuous system.
nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, ...
                'Name', 'Printer Head');

% Just get a sensible model
nlgr.SimulationOptions.AbsTol = 1e-6;
nlgr.SimulationOptions.RelTol = 1e-5;
compare(data, nlgr);

return

nlgr = setinit(nlgr, 'Fixed', false); % Estimate the initial states.
opt = nlgreyestOptions('Display', 'on');
nlgr = nlgreyest(data, nlgr, opt);
compare(data, nlgr);

  