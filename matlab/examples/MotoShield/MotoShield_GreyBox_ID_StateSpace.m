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

startScript;                                    % Clears screen and variables, except allows CI testing

%% Data preprocessing
load aprbsResult.mat                               %_Load data
Ts = 0.02;                                     %_Sample time
data = iddata([y ifltr],u,Ts,'Name','Experiment');      %_Identification data object
data = detrend(data);                           % Remove offset and linear trends
data.InputName = 'Input Voltage';                   %_Input name
data.InputUnit =  'V';                          %_Input unit
data.OutputName{1} = 'Angular Velocity';              %_Output name
data.OutputUnit{1} = 'rad/s';                          %_Output unit
data.OutputName{2} = 'Current';              %_Output name
data.OutputUnit{2} = 'A';                          %_Output unit
data.Tstart = 0;                                %_Starting time
data.TimeUnit = 's';                            %_Time unit

%% Initial guess of model parameters
J = 0.01;       %_Moment of inertia
b = 0.1;        %_Viscous friction constant
Ke = 1;     %_Electromotive force constant # Voltage over Velocity
Kt = 1;     %_Motor torque constant # Torque over Current
R = 10;         %_Electric resistance
L = 3;        %_Electric inductance

dtheta0  = data.y(1,1);        %_Initial velocity
i0 = data.y(1,2);        %_Initial current

    %_StateSpace Model
    A = [-b/J      Kt/J;                       % State-transition matrix
         -Kt/L     -R/L];
     
    B = [ 0;
        1/L];                                  % Input matrix
    
    C = [1 0;
         0 1];                                 % Output matrix
    D = [0;
         0];                                   % No direct feed-through                                            
    K = zeros(2,2);                            % Disturbance
    x0 = [dtheta0; i0];                        %_Initial condition
    disp('Initial guess:')
    sys = idss(A,B,C,D,K,x0,0)                 %_Initial State Space Model
    sys.Structure.A.Free = [1   1;             %_A variables - free
                            1   1];        
    sys.Structure.B.Free = [0  
                            1];                %_B variables
    sys.Structure.C.Free = false;              %_C fixed
    sys.DisturbanceModel = 'estimate';         % Estimate disturbance model
    sys.InitialState = 'estimate';             % Estimate initial states
    
    Options = ssestOptions;                    % State-space estimation options
    Options.Display = 'on';                    % Show progress
    Options.Focus = 'simulation';              % Focus on simulation
    Options.SearchOptions.MaxIterations = 50;  % Max iterations
    Options.InitialState = 'estimate';         % Estimate initial condition
    disp('Estimated model:')                       
    model = ssest(data,sys,Options);           % Identified Model

compare(data,model)                            % Data/Model Comparison
grid on                                       

save MotoShield_GreyboxModel_LinearSS model