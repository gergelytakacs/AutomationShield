
clc; clear; close all;

%% Data preprocessing
%load data from experiment
load('dataAprbsAll.mat')
u=dataAll(:,1)*pi/180;
y=dataAll(:,2)/1000;
%moving average filtering of y
% windowSize = 5; 
% b = (1/windowSize)*ones(1,windowSize);
% a = 1;
% y = filter(b,a,y);
 Ts = 0.01;                                     % Sampling
data = iddata(y,u,Ts,'Name','Experiment');      % Create identification data object
% subplot(211); plot(data)
 data = data(3750:3850);                         % Select a relatively stable segment
% data = data(5600:5700);
% subplot(212); plot(data)
% return
data = detrend(data);                           % Remove offset and linear trends
data.InputName = 'Servo Angle';                   % Input name
data.InputUnit =  'deg';                          % Input unit
data.OutputName = 'Ball Position';              % Output name
data.OutputUnit = 'm';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit

%% Initial guess of model parameters
%initial estimation of parameter H
m= 2.101e-3; % mass of the ball in kg
g=9.81; %gravitational acceleration 
J = 9.99e-6; % ball's moment of inertia 
R=0.004; %radius of the ball in m
H = -m*g/(J/(R^2)+m);
bb=1;
L=0.04; %m
h=(H*2*R)/L;
M=1e-3; %mass of the beam

r0  = data.y(1,1);                              % Initial ball position estimate
dr0 = (data.y(2,1)-data.y(1,1))/Ts;             % Initial ball velocity estimate
alpha0  = 30;                                        % Initial motor position estimate (after calibration)
dalpha0=0;                                          %Initial motor velocity estimate
%% Choose the type of grey-box model to identify
%model = 'nonlinear';                            % Nonlinear state-space model
model = 'linearSS';                             % Linear state-space model
%model = 'linearTF';                             % Linear transfer function

switch model
    case 'nonlinear'
    
    % Model structure
    FileName     = 'BOBShieldSin_ODE';              % File describing the model structure
    Order         = [1 1 2];                        % Model orders [ny nu nx]
 %   Parameters    = [J,R,m,M,g];            % Initial values of parameters
    Parameters    = [H];            % Initial values of parameters
 %   Parameters    = [H,bb];            % Initial values of parameters

    InitialStates = [r0;dr0];                    % Initial values of states
    Ts            = 0;                              % Time-continuous system
%%
     % Set identification options
     nlgr = idnlgrey(FileName,Order,Parameters,InitialStates,Ts,'Name','Nonlinear Grey-box Model');
     set(nlgr,'InputName','Servo Angle','InputUnit', 'deg','OutputName','Ball Position','OutputUnit','m','TimeUnit','s');
     %nlgr = setpar(nlgr,'Name',{'Supercalifragilistic Coefficient'});
     nlgr = setinit(nlgr,'Name',{'Ball Position' 'Ball Speed'});
     nlgr.InitialStates(1).Fixed = false;
     nlgr.InitialStates(2).Fixed = false;
     nlgr.Parameters(1).Fixed = false;
%      nlgr.Parameters(2).Fixed = false;
%      nlgr.Parameters(3).Fixed = false;
%      nlgr.Parameters(4).Fixed = false;

     size(nlgr);
 %%
     % Identify model
     opt = nlgreyestOptions('Display','on','EstCovar',true,'SearchMethod','Auto'); %gna
     opt.SearchOption.MaxIter = 50;                 % Maximal number of iterations
     model = nlgreyest(data,nlgr,opt);              % Run identification procedure

     case 'linearSS'
  

    
    % Model structure
%initial estimation of the system
A = [0 1 ; 0 0];
B = [0 h]';
C = [1 0];
D = [0];
                                           
    K = zeros(2,1);                            % Disturbance
    K(1) = 5;                                 % State noise

    x0 = [r0; dr0];                        % Initial condition
    disp('Initial guess:')
    sys = idss(A,B,C,D,K,x0,0)                 % Construct state-space representation

    % Mark the free parameters
    sys.Structure.A.Free = [0 0; 0 0];        
    sys.Structure.B.Free = [0  1]';         % Free and fixed variables
    sys.Structure.C.Free = false;              % No free parameters

    sys.DisturbanceModel = 'none';         % Estimate disturbance model
    sys.InitialState = 'estimate';             % Estimate initial states

    % Set identification options
    Options = ssestOptions;                    % State-space estimation options
    Options.Display = 'on';                    % Show progress
    Options.Focus = 'simulation';              % Focus on simulation
                                               % 'stability'
    Options.InitialState = 'estimate';         % Estimate initial condition
    
    % Identify model
    model = ssest(data,sys,Options);           % Run estimation procedure
    
    case 'linearTF'
    
    % Model structure
    model = idproc('P2I');              % Second order with integrator
   
    model.Structure.Kp.Value=10;         % Initial gain
    model.Structure.Kp.Maximum=10;      % Maximal gain
%    model.Structure.Kp.Minimum=5;       % Minimal gain

    model.Structure.Tp1.Value=0.2;      % Initial time constant
%     model.Structure.Tp1.Maximum=0.5;    % Maximal time constant
    model.Structure.Tp1.Minimum=0;      % Minimal time constant

    model.Structure.Tp2.Value=0.5;     % Initial time constant
%     model.Structure.Tp2.Maximum=0.9;    % Maximal time constant
    model.Structure.Tp2.Minimum=0;      % Minimal time constant

    % Set identification options
    Opt = procestOptions;               % Estimation options
    Opt.InitialCondition = 'estimate';  % Estimate the initial condition
    Opt.DisturbanceModel = 'ARMA2';     % Define noise model
    Opt.Focus = 'stability';            % Goal is to create a stable model
    Opt.Display = 'full';               % Full estimation output

    % Identify model
    model = procest(data,model,Opt);    % Estimate a process model

end

compare(data,model);                           % Compare data to model
model                                          % List model parameters
grid on                                        % Turn on grid


% Save results
save myModel.mat model      
%save BobShield_GreyboxModel_LinearSS model
%save BobShield_GreyboxModel_LinearTF model
