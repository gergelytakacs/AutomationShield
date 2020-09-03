
  
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
m = 1E-3;                                    % [kg] Magnet mass
Km= 1E-5;                                     % Magnetic constant (rhough estimate)
Ke= 1E-5;                                     % Magnetic constant (rhough estimate)
R = 200;                                      % [Ohm] Solenoid resistance
L = 0.2;                                        % [H] Solenoid inductance
u0=mean(u);                                     % [V] Input linearization around setpoint
y0=mean(y);                                     % [m] Output (position) linearization around setpoint
i0=mean(i);

    
% Initial parameters for linearized model
h0=data.y(1,1);                                 % Initial position estimate
dh0=(data.y(2,1)-data.y(1,1))/Ts;               % Initial velocity estimate
i0=data.y(1,2);                                 % Initial current estimate

y0=    0.0140;
u0=    3.6651;
i0=    0.0290;

init_sys = idgrey('MagnetoShield_ODE_Lin',[m; Km; R; L; Ke],'c',[]);
%init_sys=init(init_sys)
opt = greyestOptions('Focus','Prediction','EstimateCovariance',1)


%% Setting up paa

% Which params to estimate
%init_sys.Structure.Parameters(1).Free = [1, 1, 1, 1];


% %Estimation constraints
init_sys.Structure.Parameters(1).Minimum = [0.0001, 0, 100, 0, 0]
init_sys.Structure.Parameters(1).Maximum = [0.01, 1, 300, 1, 0.10]
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
sys = greyest(dataf,init_sys,opt)
compare(dataf,sys)
return

















%% Construct model
if fixedInductance==1                           % Magnet inductance L fixed
    A=[0       1    0;
       alpha   0   -beta
       0       0   -delta];

elseif  fixedInductance==0                      % Magnet inductance L(y) distance dependent 

    A=[0       1        0;                      % Dybamic matrix initial guess
       alpha   0       -beta
       0       gamma   -delta];     
end

B=[0; 0; epsilon];                              % Input matrix
C=[1 0 0;                                       % Output matrix
   0 0 1];                                      % Distance and current measured
D=[0; 0];                                       % No feed-through                                            
K = zeros(3,2);                                 % Disturbance
x0=[h0; dh0; i0];                               % Initial condition
disp('Initial guess:')
sys=idss(A,B,C,D,K,x0,0)                        % Construct state-space representation


% Mark the free parameters
if fixedInductance==1                           % Magnet inductance L fixed
    sys.Structure.A.Free=      [0     0    0;   % Free and fixed variables
                                1     0    1;
                                0     0    1];
elseif fixedInductance==0                       % Magnet inductance L(y) distance dependent
    sys.Structure.A.Free=      [0     0    0;   % Free and fixed variables
                                1     0    1;
                                0     1    1];
end
                      
sys.Structure.B.Free=  [0  0 1]';               % Free and fixed variables
sys.Structure.C.Free=  false;                   % No free parameters

sys.DisturbanceModel = 'estimate';              % Estimate disturbance model
sys.InitialState = 'estimate';                  % Estimate initial states

%% Set estimation options
Options = ssestOptions;                         % State-space estimation options
Options.Display = 'on';                         % Show progress
Options.Focus = 'simulation';                   % Identification focus
Options.EnforceStability = 0;                   % Unstable model
Options.InitialState = 'estimate';              % Estimate initial condition

%% Estimate and list parameters
disp('Identified model:')
model = ssest(dataf,sys,Options)                % Launch estimation procedure
compare(dataf,model)                            % Compare data to model

disp('---Parameter Comparison---')
Ld=['L (measured): ',num2str(L),' [H],   L (model): ',num2str(1/model.b(3),3),' H'];
disp(Ld)
Rd=['R (measured): ',num2str(R),' [Ohm], R (model): ',num2str(-model.a(3,3)/model.b(3),3),' [Ohm]'];
disp(Rd)
if fixedInductance==0 
   
Ke=mean([model.a(2,1)*2*m*y0^3/u0^2 -(model.a(2,3)*2*m*y0^2)/u0]);
Kstd=std([model.a(2,1)*2*m*y0^3/u0^2 -(model.a(2,3)*2*m*y0^2)/u0]);
Kd=['K (guess): ',num2str(Km), ',    K (model,mean): ',num2str(Ke,1),' +/- ',num2str(Kstd,1)];
disp(Kd)
disp(['K (range): ',num2str([model.a(2,1)*2*m*y0^3/u0^2 -(model.a(2,3)*2*m*y0^2)/u0])])
end


