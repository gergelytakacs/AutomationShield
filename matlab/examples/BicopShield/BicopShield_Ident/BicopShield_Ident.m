clear all; close all; clc;
%% Data preprocessing
readtable pid1.csv ;                             %_Load data
down_const = 5;
y=downsample(ans.Var1,down_const);              % Output
ref=ans.Var2;                                   % Reference
u1=downsample(ans.Var3,down_const);             % motor1 input
u2=downsample(ans.Var4,down_const);             % motor2 input
u=ans.Var5;                                     % PID output

u1=0.05*u1;
u2=0.05*u2;

Ts = 0.01*down_const;                           %_Sample time
y_rad=deg2rad(y);
y_v=gradient(y_rad,Ts);
%%
data = iddata([lowpass(y_rad,0.001), lowpass(y_v,0.001)],[lowpass(u1.^2,0.001),lowpass(u2.^2,0.001)],Ts,'Name','Experiment');      %_Identification data object
data = data(2600:3000);
% data.InputName = {'Input Voltage Motor 1','Input Volage Motor 2'};   %_Input name
% data.InputUnit =  {'V','V'};                          %_Input unit
% data.OutputName = {'Angle'};              %_Output name
% data.OutputUnit = {'Degree'};        %_Output unit
% data.Tstart = 0;                                %_Starting time
% data.TimeUnit = 's';                            %_Time unit

%% Initial guess of model parameters
J = 0.5;       %_Moment of inertia
b = 0.23;        %_Viscous friction constant
l1= 0.125;       % Length of arm 1
l2= 0.125;       % Length of arm 2
k=0.12  ;        % Wire 
k1 = 0.28;       % Motor constant

    %_StateSpace Model
    A = [0      1;                       % State-transition matrix
         k/J     -b/J];
     
    B = [ 0     0;
        k1*l1/J   -k1*l2/J];                                  % Input matrix
    
    C = [1 0; 0 1];                                 % Output matrix
    D = [0 0; 0 0];                                 % No direct feed-through                                                                             
    K = zeros(2,2);                            % Disturbance
    x0 = [0,0];                               %_Initial condition
    disp('Initial guess:')
    sys = idss(A,B,C,D,K,x0,0.0);  %continuous               %_Initial State Space Model
    sys.Structure.A.Free = [0   0;             %_A variables - free
                            1   1];        
    sys.Structure.B.Free = [0 0;  
                            1 1];                %_B variables
    sys.Structure.C.Free = false;              %_C fixed
    sys.Structure.D.Free = false;              %_D fixed
    sys.Structure.A.Minimum = [0 0 ;
                                -inf -10];
    sys.Structure.B.Minimum = [0 0;
                                0.0 -inf];
    sys.DisturbanceModel = 'none';         % Estimate disturbance model
    sys.InitialState = 'estimate';             % Estimate initial states
    compare(data,sys)
    %%
    Options = ssestOptions;                    % State-space estimation options
    Options.InputInterSample = 'zoh';
    Options.EnforceStability = true;
    Options.Display = 'on';                    % Show progress
    Options.Focus = 'simulation';              % Focus on simulation
    Options.SearchOptions.MaxIterations = 300;  % Max iterations
    Options.SearchMethod = 'auto';
    Options.InitialState = 'auto';         % Estimate initial condition
    
    Options.N4Weight = 'MOESP';
    Options.N4Horizon = [50 50 50];
    disp('Estimated model:')                       
    model = ssest(data,sys,Options);           % Identified Model
   %%
comopt = compareOptions('InitialCondition','z');
figure
compare(data,model)                            % Data/Model Comparison
grid on                                       

save BicopShield_GreyboxModel_LinearSS model
%%


[xhat, yhat] = estimateKalmanState(U(:,k), Y, A, B, C, Q_Kalman, R_Kalman, xICWhole);