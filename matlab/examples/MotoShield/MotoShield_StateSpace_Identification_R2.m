%   MotoShield System Identification example
% 
%   This is an example of MotoShield System Identification.
%   two modes of state-space model identification. Blackbox and whitebox
%   models are available.
%   
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created Ján Boldocký.




startScript;                                    % Clears screen and variables, except allows CI testing

%% Data preprocessing
                           %_Load data
load identification_data.mat
Ts = 0.02;                                     %_Sample time
y=y/Ts*2*pi/28;                         %_PPS to rad/sec conversion
fs=1e5;
[ifltr,lp_filter]=lowpass(i,0.05,fs,'StopbandAttenuation',10,'Steepness',0.95);
%freqz(lp_filter)
figure
plot(ifltr)
trenddata = iddata([y ifltr],u,Ts,'Name','Experiment');      %_Identification data object
data=trenddata(200:3000);
%data = detrend(trenddata(500:2500));                           % Remove offset and linear trends
data.InputName = 'Input Voltage';                   %_Input name
data.InputUnit =  'V';                          %_Input unit
data.OutputName{1} = 'Angular Velocity';              %_Output name
data.OutputUnit{1} = 'rad/s';                          %_Output unit
data.OutputName{2} = 'Current';              %_Output name
data.OutputUnit{2} = 'A';                          %_Output unit
data.Tstart = 200*Ts;                                %_Starting time
data.TimeUnit = 's';                            %_Time unit

%% Initial guess of model parameters
mode = 'greybox';
switch mode
    case 'greybox'
J = 1e-4;       %_Moment of inertia
b = 1e-4;        %_Viscous friction constant
Ke = 1-2;     %_Electromotive force constant # Voltage over Velocity
Kt = 1-2;     %_Motor torque constant # Torque over Current
R = 10;         %_Electric resistance
L = 3500e-6;        %_Electric inductance
dtheta0  = data.y(1,1);        %_Initial velocity
i0 = data.y(1,2);        %_Initial current
alpha=b/J;
betta=Kt/J;
gamma=Kt/L;
delta=R/L;
epsilon=1/L;
    %_StateSpace Model
    A = [-alpha      betta;                       % State-transition matrix
         -gamma     -delta];    
    B = [ 0;
        epsilon];                                  % Input matrix
    C = [1 0;
         0 1];                                 % Output matrix
    D = [0;
         0];                                   % No direct feed-through                                            
    K = zeros(2,2);                            % Disturbance
    x0 = [dtheta0; i0];                        %_Initial condition
    disp('Initial guess:')
    sys = idss(A,B,C,D,K,x0,0)                 %_Initial State Space Model
    
    sys.Structure.A.Free = [1   1;             %_A variables - free
                            1   0];        
    sys.Structure.B.Free = [0  
                            0];                %_B variables
    sys.Structure.C.Free = false;              %_C fixed
    sys.Structure.A.Minimum = [-inf 0;
                              -inf -inf];
    sys.Structure.A.Maximum = [0 inf;
                               0 0];
    sys.DisturbanceModel = 'estimate';         % Estimate disturbance model
    sys.InitialState = 'estimate';             % Estimate initial states
    Options = ssestOptions;                    % State-space estimation options
    Options.Display = 'on';                    % Show progress
    Options.Focus = 'simulation';              % Focus on simulation
    Options.SearchOptions.MaxIterations = 300;  % Max iterations
    Options.InitialState = 'estimate';         % Estimate initial condition
    disp('Estimated model:')                       
    model = ssest(data,sys,Options);           % Identified Model

    case 'blackbox'
J = 1e-4;       %_Moment of inertia
b = 1e-4;        %_Viscous friction constant
Ke = 1;     %_Electromotive force constant # Voltage over Velocity
Kt = 1;     %_Motor torque constant # Torque over Current
R = 10;         %_Electric resistance
L = 3500e-6;        %_Electric inductance
dtheta0  = data.y(1,1);        %_Initial velocity
i0 = data.y(1,2);        %_Initial current
alpha=b/J;
betta=Kt/J;
gamma=Kt/L;
delta=R/L;
epsilon=1/L;
    %_StateSpace Model
    A = [-alpha      betta;                       % State-transition matrix
         -gamma     -delta];    
    B = [ 0;
        epsilon];                                  % Input matrix
    C = [1 0;
         0 1];                                 % Output matrix
    D = [0;
         0];                                   % No direct feed-through                                            
    K = zeros(2,2);                            % Disturbance
    x0 = [dtheta0; i0];                        %_Initial condition
    disp('Initial guess:')
    sys = idss(A,B,C,D,K,x0,0)                 %_Initial State Space Model
    
    sys.DisturbanceModel = 'estimate';         %-Estimate disturbance model
    sys.InitialState = 'estimate';             %-Estimate initial states
    Options = ssestOptions;                    %-State-space estimation options
    Options.Display = 'on';                    
    Options.Focus = 'simulation';              
    Options.SearchOptions.MaxIterations = 300;  %-Max iterations
    Options.InitialState = x0;         %-Estimate initial condition
    disp('Estimated model:')                       
    model = ssest(data,sys,Options);           %-Identified Model
end
zmodel=c2d(model,Ts,'zoh');
compare(data,model)
grid on                                       
save MotoShield_LinearSS model zmodel
%% Parameters back-calculation
Lm=1/model.B(2)
Rm=-model.A(2,2)*Lm
Km=-model.A(2,1)*Lm
Jm=Km/model.A(1,2)
bm=-model.A(1,1)*Jm
