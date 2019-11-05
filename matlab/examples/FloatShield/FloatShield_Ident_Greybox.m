%   FloatShield grey-box identification procedure
% 
%   Identification of a first-principles based grey-box model using experimental data.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Martin Gulan and Gergely Takács. 

clc; clear; close all;

%% Data preprocessing
load resultID.mat                               % Read identificaiton results
% load aprbs.mat
Ts = 0.025;                                     % Sampling
data = iddata(y,u,Ts,'Name','Experiment');      % Create identification data object
% subplot(211); plot(data)
data = data(4530:5151);                         % Select a relatively stable segment
% data = data(1200:2500);
% subplot(212); plot(data)
% return
data = detrend(data);                           % Remove offset and linear trends
data.InputName = 'Fan Power';                   % Input name
data.InputUnit =  '%';                          % Input unit
data.OutputName = 'Ball Position';              % Output name
data.OutputUnit = 'm';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit

%% Initial guess of model parameters
m = 0.00384;                                    % [kg] Weight of the ball
r = 0.03;                                       % [m] Radius of the ball
cd = 0.74;                                      % [-] Drag coefficient
ro = 1.23;                                      % [kg/m3] Density of air
% A = pi*r^2;                                     % [m2] Exposed area of the ball; circle's...
A = 2*pi*r^2;                                   % ...or half of a sphere's area
g = 9.81;                                       % [m/s2] Gravitataional acceleration
k = 7.78;                                       % Fan's gain
tau = 0.14;                                     % Fan's time constant

tau2 = 0.6;                                     % Ball's time constant

h0  = data.y(1,1);                              % Initial ball position estimate
dh0 = (data.y(2,1)-data.y(1,1))/Ts;             % Initial ball velocity estimate
v0  = 0;                                        % Initial airspeed estimate

%% Choose the type of grey-box model to identify
model = 'nonlinear';                            % Nonlinear state-space model
% model = 'linearSS';                             % Linear state-space model
% model = 'linearTF';                             % Linear transfer function

switch model
    case 'nonlinear'
    
    % Model structure
    FileName      = 'FloatShield_ODE';              % File describing the model structure
    Order         = [1 1 3];                        % Model orders [ny nu nx]
    Parameters    = [m;cd;ro;A;g;k;tau];            % Initial values of parameters
    InitialStates = [h0;dh0;v0];                    % Initial values of states
    Ts            = 0;                              % Time-continuous system

    % Set identification options
    nlgr = idnlgrey(FileName,Order,Parameters,InitialStates,Ts,'Name','Nonlinear Grey-box Model');
    set(nlgr,'InputName','Fan Power','InputUnit', '%','OutputName','Ball Position','OutputUnit','m','TimeUnit','s');
    nlgr = setpar(nlgr,'Name',{'Ball Mass' 'Drag Coefficient' 'Air Density' 'Exposed Area' ...
                               'Gravitational Acceleration' 'Fan Gain' 'Fan Time Constant'});
    nlgr = setinit(nlgr,'Name',{'Ball Position' 'Ball Speed' 'Air Speed'});
    % nlgr.Parameters(1).Minimum = 0.0035;
    % nlgr.Parameters(1).Maximum = 0.004;
    nlgr.Parameters(1).Fixed = true;
    nlgr.Parameters(2).Minimum = 0.3;
    nlgr.Parameters(2).Maximum = 1.0;
    % nlgr.Parameters(3).Minimum = 1.0;
    nlgr.Parameters(3).Fixed = true;
    % nlgr.Parameters(4).Minimum = 0.0028;
    % nlgr.Parameters(4).Maximum = 0.0057;
    nlgr.Parameters(4).Fixed = true;
    nlgr.Parameters(5).Fixed = true;
    nlgr.Parameters(6).Minimum = 0;
    nlgr.Parameters(7).Minimum = 0;
    nlgr.InitialStates(3).Fixed = false;
    size(nlgr)

    % Identify model
    opt = nlgreyestOptions('Display','on','EstCovar',true,'SearchMethod','Auto'); %gna
    opt.SearchOption.MaxIter = 50;                 % Maximal number of iterations
    model = nlgreyest(data,nlgr,opt);              % Run identification procedure

    case 'linearSS'
    
    % Unknonwn parameters
    alpha = 1/tau;
    beta  = 1/tau2;
    gamma = k/tau;
    
    % Model structure
    A = [0     1      0;                         % State-transition matrix
         0 -beta   beta;
         0     0 -alpha];
    B = [    0;
         gamma;
             0];                               % Input matrix
    C = [1 0 0];                               % Output matrix
    D = 0;                                     % No direct feed-through                                            
    K = zeros(3,1);                            % Disturbance
    K(2) = 25;                                 % State noise

    x0 = [h0; dh0; v0];                        % Initial condition
    disp('Initial guess:')
    sys = idss(A,B,C,D,K,x0,0)                 % Construct state-space representation

    % Mark the free parameters
    sys.Structure.A.Free = [0  0  0;           % Free and fixed variables
                            0  1  1;
                            0  0  1];        
    sys.Structure.B.Free = [0  1  0]';         % Free and fixed variables
    sys.Structure.C.Free = false;              % No free parameters

    sys.DisturbanceModel = 'estimate';         % Estimate disturbance model
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

    model.Structure.Kp.Value=k;         % Initial gain
    model.Structure.Kp.Maximum=10;      % Maximal gain
    model.Structure.Kp.Minimum=5;       % Minimal gain

    model.Structure.Tp1.Value=tau;      % Initial time constant
%     model.Structure.Tp1.Maximum=0.5;    % Maximal time constant
    model.Structure.Tp1.Minimum=0;      % Minimal time constant

    model.Structure.Tp2.Value=tau2;     % Initial time constant
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

return
% Save results
save FloatShield_GreyboxModel_Nonlinear.mat model      
save FloatShield_GreyboxModel_LinearSS model
save FloatShield_GreyboxModel_LinearTF model
