%   MotoShield grey-box identification procedure
% 
%   Identification of a grey-box model using experimental data.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács and Ján Boldocký. 

clc; clear; close all;

%% Data preprocessing
load aprbsResult.mat                               % Read identificaiton results
Ts = 0.02;                                     % Sampling
data = iddata([y ifltr],u,Ts,'Name','Experiment');      % Create identification data object
data = detrend(data);                           % Remove offset and linear trends
data.InputName = 'Input Voltage';                   % Input name
data.InputUnit =  'V';                          % Input unit
data.OutputName{1} = 'Angular Velocity';              % Output name
data.OutputUnit{1} = 'rad/s';                          % Output unit
data.OutputName{2} = 'Current';              % Output name
data.OutputUnit{2} = 'A';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit

%% Initial guess of model parameters
J = 0.01;       %-Moment of inertia
b = 0.1;        %-Viscous friction constant
Ke = 1;     %-Electromotive force constant # Voltage over Velocity
Kt = 1;     %-Motor torque constant # Torque over Current
R = 10;         %-Electric resistance
L = 3;        %-Electric inductance

dtheta0  = data.y(1,1);        %-Initial velocity
i0 = data.y(1,2);        %-Initial current

    % Linear SS Model
    A = [-b/J      Kt/J;                       % State-transition matrix
         -Kt/L     -R/L];
     
    B = [ 0;
        1/L];                                  % Input matrix
    
    C = [1 0;
         0 1];                                 % Output matrix
    D = [0;
         0];                                   % No direct feed-through                                            
    K = zeros(2,2);                            % Disturbance
    x0 = [dtheta0; i0];                        % Initial condition
    disp('Initial guess:')
    sys = idss(A,B,C,D,K,x0,0)                 % Construct state-space representation
    % Mark the free parameters
    sys.Structure.A.Free = [1   1;             % Free and fixed variables
                            1   1];        
    sys.Structure.B.Free = [0  1]';            % Free and fixed variables # Transposed
    sys.Structure.C.Free = false;              % No free parameters
    sys.DisturbanceModel = 'estimate';         % Estimate disturbance model
    sys.InitialState = 'estimate';             % Estimate initial states

    % Set identification options
    Options = ssestOptions;                    % State-space estimation options
    Options.Display = 'on';                    % Show progress
    Options.Focus = 'simulation';              % Focus on simulation
    Options.SearchOptions.MaxIterations = 50;
    Options.InitialState = 'estimate';         % Estimate initial condition
    disp('Estimated model:')                        % Identified model    
    % Identify model
    model = ssest(data,sys,Options);           % Run estimation

compare(data,model) 
grid on                                        % Turn on grid

save MotoShield_GreyboxModel_LinearSS model
