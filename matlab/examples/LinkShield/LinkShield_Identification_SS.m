clear; close all; clc

%% Load measured data 
load LinkShield_ID_Data.mat

% Cut most relevant data
Alpha = Alpha(24:300);
Theta = Theta(24:300);
U = U(24:300);

% Sampling period
Ts = 0.005;                 

%% Create identification data object
data = iddata([Theta,Alpha], U, Ts, 'Name', 'Experiment');
%data = detrend(data);
data.InputName = 'Voltage';
data.InputUnit = 'V';
data.OutputName = {'Servo Angle','Beam Angle'};
data.OutputUnit = {'Rad','Rad'};
data.Tstart = 0;
data.TimeUnit = 's';

%% Initial states
Alpha0 = data.y(1,2);
dAlpha0 = (data.y(2,2) - data.y(1,1))/Ts;
Theta0 = data.y(1,1);
dTheta0 = (data.y(2,1) - data.y(1,1))/Ts;

% Alpha0 = 0;
% dAlpha0 = 0;
% Theta0 = 0;
% dTheta0 = 0;

% Best match

Beq = 0.004;
Jeq =0.10000;
Bl = 0.00002;
%Jl = 0.5;
pomer=1.00001; %(Jl+Jeq)/Jl
Ks = 0.75;
c1=0.0; %z prevodu torque na voltage
c2=1.0; %z prevodu torque na voltage



% A = [0 0 1 0;
%     0 0 0 1;
%     0 Ks/Jeq ((-Beq/Jeq)-c1) Bl/Jeq;
%     0 -Ks*pomer/Jeq ((Beq/Jeq)-c1)/Jeq Bl/Jeq]

% A = [0 0 1 0;
%     0 0 0 1;
%     0 Ks/Jeq -Beq/Jeq Bl/Jeq;
%     0 -Ks*((Jeq+Jl)/(Jeq*Jl)) Beq/Jeq Bl*((Jl+Jeq)/(Jl*Jeq))];


% B = [0 0 c2/Jeq -c2/Jeq]';

C = eye(2,4);
D = zeros(2,1);
K = zeros(4,2); 


A = [0 0 1 0;
     0 0 0 1;
     0    2746  -127.25   0;
     0    -7804  227.066  0];
B = [0;
     0;
     50.84
     -50.333];


x0=[Theta0,Alpha0,dTheta0,dAlpha0]; 
sys=idss(A,B,C,0,K,x0,0);  

sys.Structure.A.Free=  [0  0  0  0;
                        0  0  0  0;
                        0  1  1  0;
                        0  1  1  0];

sys.Structure.A.Minimum = [0 0 0 0;
                           0 0 0 0;
                           0 -inf -inf 0;
                           0 -inf -inf -inf];

sys.Structure.A.Maximum = [0 0 0 0;
                           0 0 0 0;
                           0 inf inf inf;
                           0 inf inf inf];

sys.Structure.B.Free=  [0  0  1  1]'; 
% sys.Structure.B.Minimum=  [0  0  -inf  -inf]'; 
% sys.Structure.B.Free=  [0  0  inf  0]'; 

% sys.Structure.A.Free=  true;
% sys.Structure.B.Free=  true; 



sys.Structure.C.Free=  false;                   % No free parameters
sys.Structure.D.Free=  false;                   % No free parameters
sys.Structure.K.Free=  false;                   % No free parameters
sys.DisturbanceModel = 'none';                  % Don't estimate disturbance

Options = ssestOptions;                         % State-space estimation options
Options.Display = 'off';                        % Show progress
Options.Focus = 'simulation';                   % Identification focus
Options.EnforceStability = 1;                   % Unstable model
Options.InitialState = 'estimate';              % Estimate initial condition
Options.SearchOptions.MaxIterations=100;        % Set maximum count of iteration
Options.SearchMethod='gn';                      % Choose search method

disp('Identified model:')
model = ssest(data,sys,Options);                 % Launch estimation procedure
display(model)

compare(data,model,1)                           % Compare data to model (1-step prediction simulation)

%% Simulacia

dmodel=c2d(model,Ts);
dA=dmodel.A;
dB=dmodel.B;
dC=dmodel.C;

x0=[Theta0; Alpha0; dTheta0; dAlpha0 ]; 
x(:,1)=x0;
y = zeros(2,length(U));

for i=1:length(U)
    x(:,i+1) = dA*x(:,i)+dB*U(i);
    y(:,i) = dC*x(:,i);
end
figure('Name','Simulation')

subplot(2,1,1)
plot(y(1,:)); 
hold on
plot(Theta)

subplot(2,1,2)
plot(y(2,:)); 
hold on
plot(Alpha)

save('LinkShield_SSID.mat','model','Ts')