
  
clc; clear all;                                 % Clears screen and all variables

fixedInductance=0;                              % Fixed or distance dependent inductance?
                                                % to mathematical model
load('IDExperiment1');
Ts=0.003250;                                       % [s] Sampling
y=result(:,1)/1000;                             % [m] Output in meters
u=result(:,2);                                  % [V] Input is closed loop + probe signal
i=result(:,3)/1000;                             % [A] Current

%% System identification data object
data = iddata([y i],u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid';                    % Input name: Solenoid voltage
data.InputUnit =  'V';                          % Input unit
data.OutputName{1} = 'Position';                % Output 1 name
data.OutputUnit{1} = 'm';                       % Output 1 unit
data.OutputName{2} = 'Current';                 % Output 2 name
data.OutputUnit{2} = 'A';                       % Output 2 unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(100:end);                           % Discard the time when magnet is on ground, pick close to linearization point
data=detrend(data);                             % Discard offset                               
dataf = fft(data);                              % Frequency domain 

%% Parameters
%% Parameters
m = 0.76E-3;                                    % [kg] Magnet mass
R = 198.3;                                      % [Ohm] Solenoid resistance
L = 0.239;                                      % [H] Solenoid inductance
u0=mean(u);                                     % [V] Input linearization around setpoint
y0=mean(y);                                     % [m] Output (position) linearization around setpoint
Km=2E-6;                                        % Magnetic constant (rhough estimate)
    
% Initial parameters for linearized model
h0=data.y(1,1);                                 % Initial position estimate
dh0=(data.y(2,1)-data.y(1,1))/Ts;               % Initial velocity estimate
i0=data.y(1,2);                                 % Initial current estimate
alpha=2*(Km/m)*u0^2/y0^3;                       % Linearized parameter guess
beta=2*(Km/m)*u0/y0^2;                          % Linearized parameter guess
gamma=2*Km*mean(i)/(L*y0^2);                    % Linearized parameter guess
delta=R/L;                                      % Parameter guess
epsilon=1/L;                                    % Parameter guess

alpha=2.573e+04;                       % Linearized parameter guess
beta=98.32;                          % Linearized parameter guess
gamma=0.001441 ;                    % Linearized parameter guess
delta=829.7 ;                                      % Parameter guess
epsilon= 4.184;                                    % Parameter gues

init_sys = idgrey('MagnetoShield_ODE_Lin',[alpha;beta;gamma;delta;epsilon],'c',[]);


init_sys.DisturbanceModel = 'estimate';              % Estimate disturbance model
init_sys.InitialState = 'estimate';                  % Estimate initial states


%init_sys=init(init_sys)
Options = greyestOptions;
Options.Display = 'on';                         % Show progress
Options.Focus = 'simulation';                   % Identification focus
Options.EnforceStability = 0;                   % Unstable model
Options.InitialState = 'estimate';              % Estimate initial condition


%% Setting up paa

% Which params to estimate
%init_sys.Structure.Parameters(1).Free = [1, 1, 1, 1];


% %Estimation constraints
%init_sys.Structure.Parameters(1).Minimum = [0.0001, 0, 100, 0, 0]
%init_sys.Structure.Parameters(1).Maximum = [0.01, 1, 300, 1, 0.10]
% init_sys.Parameters(1).Maximum = 1E-7;
% init_sys.Parameters(2).Minimum = m/1.5;    % 10% change allowed
% init_sys.Parameters(2).Maximum = m*1.5;    % 10% change allowed
% init_sys.Parameters(3).Minimum = R/1.5;    % 10% change allowed
% init_sys.Parameters(3).Maximum = R*1.5;    % 10% change allowed
% init_sys.Parameters(4).Minimum = L/1.5;    % 50% change allowed
% init_sys.Parameters(4).Maximum = L*1.5;    % 50% change allowed
% 
% 
% init_sys.InitialStates(1).Fixed   = false; 
% init_sys.InitialStates(2).Fixed   = false; 
% init_sys.InitialStates(3).Fixed   = false; 
sys = greyest(dataf,init_sys,Options)
compare(dataf,sys)
return







