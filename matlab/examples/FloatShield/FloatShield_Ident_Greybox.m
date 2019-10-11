%   FloatShield grey-box identification procedure
%
%   Grey box identification of either a first-principles based nonlinear
%   model, linear third-order state-space model or linear second order
%   transfer function with an integrator, using experimental data.
%
%   The example takes input-output identification results from the
%   FloatShield device, where input is power to the fan in percents and
%   output is the altitude (height) of the ball. Based on the models of
%   Chacon et al. (2017) and Chaos et al. (2019) the example identifies
%   selected model based on measured data.
%
%   References:
%   Chacon, J., Saenz, J., Torre, L., Diaz, J. M., & Esquembre, F. (2017).
%       Design of a Low-Cost Air Levitation System for Teaching Control
%       Engineering. Sensors (Basel, Switzerland), 17(10), 2321.
%       doi:10.3390/s17102321
%   Chaos, D., Chacón, J., Aranda-Escolástico, E., Dormido, S. (2019).
%   Robust switched control of an air levitation system with minimum
%   sensing. ISA Transactions (2019)
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Martin Gulan and Gergely Takács.
%   Last update: 10.10.2019.

clc;clear;close all;                            % Clear screen and memory

%% Data preprocessing
load resultID.mat                               % Read experiment results
Ts = 0.025;                                     % Sampling period [s]
data = iddata(y, u, Ts, 'Name', 'Experiment');  % Create identification data object
figure; subplot(1,2,1); plot(data); title('Raw Input-Output Data'); grid on
data = data(4530:5151);                         % Select a relatively stable segment
subplot(1,2,2); plot(data); title('Used Input-Output Data'); grid on

data = detrend(data);                  % Remove offset and linear trends
data.InputName = 'Fan Power';          % Input name
data.InputUnit = '%';                  % Input unit
data.OutputName = 'Ball Position';     % Output name
data.OutputUnit = 'mm';                % Output unit
data.Tstart = 0;                       % Starting time
data.TimeUnit = 's';                   % Time unit

%% Initial guess of model parameters
m = 0.00384;                % [kg] Weight of the ball
r = 0.015;                  % [m] Radius of the ball
cd = 0.74;                  % [-] Drag coefficient
ro = 1.23;                  % [kg/m3] Density of air
% A = pi*r^2;               % [m2] Exposed area of the ball; circle's...
A = 2 * pi * r^2;           % ...or half of a sphere's area
g = 9.81;                   % [m/s2] Gravitataional acceleration
k = 7.78;                   % Fan's gain
tau = 0.14;                 % Fan's time constant

tau2 = 0.6;                 % Ball's time constant

h0 = data.y(1, 1);                          % Initial ball position estimate
dh0 = (data.y(2, 1) - data.y(1, 1)) / Ts;   % Initial ball velocity estimate
v0 = 0;                                     % Initial airspeed estimate

%% Identification of chosen type grey-box model
modelType = 'nonlinear';                    % Nonlinear state-space model
% modelType = 'linearSS';                   % Linear state-space model
% modelType = 'linearTF';                   % Linear transfer function

switch modelType    
    case 'nonlinear'

        % Model structure
        FileName = 'FloatShield_ODE';           % File describing the model structure
        Order = [1, 1, 3];                      % Model orders [ny nu nx]
        Parameters = [m; cd; ro; A; g; k; tau]; % Initial values of parameters
        InitialStates = [h0; dh0; v0];          % Initial values of states
        Ts = 0;                                 % Time-continuous system

        % Set identification options
        nlgr = idnlgrey(FileName, Order, Parameters, InitialStates, Ts, 'Name', 'Nonlinear Grey-box Model');
        set(nlgr, 'InputName', 'Fan Power', 'InputUnit', '%', 'OutputName', 'Ball Position', 'OutputUnit', 'mm', 'TimeUnit', 's');
        nlgr = setpar(nlgr, 'Name', {'Ball Mass', 'Drag Coefficient', 'Air Density', 'Exposed Area', ...
            'Gravitational Acceleration', 'Fan Gain', 'Fan Time Constant'});
        nlgr = setinit(nlgr, 'Name', {'Ball Position', 'Ball Speed', 'Air Speed'});
        nlgr.Parameters(1).Fixed = true;
        %nlgr.Parameters(1).Minimum = 0.0035;
        %nlgr.Parameters(1).Maximum = 0.004;
        nlgr.Parameters(2).Minimum = 0.3;
        nlgr.Parameters(2).Maximum = 1.0;        
        nlgr.Parameters(3).Fixed = true;        
        nlgr.Parameters(4).Fixed = true;
        nlgr.Parameters(5).Fixed = true;
        nlgr.Parameters(6).Minimum = 0;
        nlgr.Parameters(7).Minimum = 0;
        nlgr.InitialStates(3).Fixed = false;
        size(nlgr)
        
        % Identify model
        opt = nlgreyestOptions('Display', 'on', 'EstCovar', true, 'SearchMethod', 'Auto');
        opt.SearchOption.MaxIter = 50;       % Maximal number of iterations
        model = nlgreyest(data, nlgr, opt);  % Run identification procedure

        
    case 'linearSS'

        % Unknonwn parameters
        alpha = 1 / tau;
        beta = 1 / tau2;
        gamma = k / tau;

        % Model structure
        A = [-alpha, 0, 0; ...           % State-transition matrix
                  0, 0, 1; ...
               beta, 0, -beta];
        B = [gamma; 0; 0];               % Input matrix
        C = [0, 1, 0];                   % Output matrix
        D = 0;                           % No direct feed-through
        K = zeros(3, 1);                 % Disturbance
        K(2) = 25;                       % State noise

        x0 = [v0; h0; dh0];              % Initial condition
        disp('Initial guess:')
        sys = idss(A, B, C, D, K, x0, 0) % Construct state-space representation

        % Mark the free parameters
        sys.Structure.A.Free = [1, 0, 0; ... % Free and fixed variables
                                0, 0, 0; ...
                                1, 0, 1];
        sys.Structure.B.Free = [1; 0; 0];    % Free and fixed variables
        sys.Structure.C.Free = false;        % No free parameters

        sys.DisturbanceModel = 'estimate';   % Estimate disturbance model
        sys.InitialState = 'estimate';       % Estimate initial states

        % Set identification options
        Options = ssestOptions;            % State-space estimation options
        Options.Display = 'on';            % Show progress
        Options.Focus = 'simulation';      % Focus on simulation        
        Options.InitialState = 'estimate'; % Estimate initial condition

        % Identify model
        model = ssest(data, sys, Options); % Run estimation procedure

        
    case 'linearTF'

        % Model structure
        model = idproc('P2I');              % Second order with integrator

        model.Structure.Kp.Value = k;       % Initial gain
        model.Structure.Kp.Maximum = 10;    % Maximal gain
        model.Structure.Kp.Minimum = 5;     % Minimal gain

        model.Structure.Tp1.Value = tau;    % Initial time constant
        %model.Structure.Tp1.Maximum=0.5;   % Maximal time constant
        model.Structure.Tp1.Minimum = 0;    % Minimal time constant

        model.Structure.Tp2.Value = tau2;   % Initial time constant
        %model.Structure.Tp2.Maximum=0.9;   % Maximal time constant
        model.Structure.Tp2.Minimum = 0;    % Minimal time constant

        % Set identification options
        Opt = procestOptions;               % Estimation options
        Opt.InitialCondition = 'estimate';  % Estimate the initial condition
        Opt.DisturbanceModel = 'ARMA2';     % Define noise model
        Opt.Focus = 'stability';            % Goal is to create a stable model
        Opt.Display = 'full';               % Full estimation output

        % Identify model
        model = procest(data, model, Opt);  % Estimate a process model
end

figure; compare(data, model); grid on       % Compare data to model
model                                       % List model parameters                                    

return
switch modelType                            % Save results
    case 'nonlinear'
        save FloatShield_GreyboxModel_Nonlinear model
    case 'linearSS'
        save FloatShield_GreyboxModel_LinearSS model
    case 'linearTF'
        save FloatShield_GreyboxModel_LinearTF model
end