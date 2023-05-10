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
c1=1.0; %z prevodu torque na voltage
c2=1.0; %z prevodu torque na voltage



% A = [0 0 1 0;
%     0 0 0 1;
%     0 Ks/Jeq ((-Beq/Jeq)-c1) Bl/Jeq;
%     0 -Ks*pomer/Jeq ((Beq/Jeq)-c1)/Jeq Bl/Jeq]

% A = [0 0 1 0;
%      0 0 0 1;
%      0 Ks/Jeq -(Beq-c2)/Jeq 0;
%      0 -Ks*((Jeq+Jl)/(Jeq*Jl)) (Beq-c2)/Jeq 0];
% B = [0 0 c1/Jeq -c1/Jeq]';
C = eye(2,4);
D = zeros(2,1);
K = zeros(4,2); 


A = [0 0 1 0;
     0 0 0 1;
     0    2746  -12.25   0;
     0    -7804  22.066  0];
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
                           0 -inf -inf -inf;
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
x(:,1)=model(1, 1).Report.Parameters.X0  ;

Uval = ones(277,1)*5;
Uval(1:10)=0;
Uval=U;
y = zeros(2,length(Uval));

R_Kalman = [5.808465952461661e-03 0;
            0 1.266009518986769e-07];

Q_Kalman = diag([50 20 1 1]);


for i=1:length(Uval)

%     Y(1,1) = Theta(i);  
%     Y(2,1) = Alpha(i);
% 
%  [xhat(:,i), yhat(:,i)] = estimateKalmanState(U(i), Y, dA, dB, dC, Q_Kalman, R_Kalman);

x(:,i+1) = dA*x(:,i) + dB*Uval(i);
y(:,i+1) = dC*x(:,i+1); 


 
end
%%
tplot = 0:5:299*5;
tplot = t/1000;



subplot(3,1,1)
stairs(t,U,LineWidth=1.5)
legend("U")
title("Priebeh identifikačného experimentu")
xticks(0:0.25:1.5)
ylim([-0.2 5.2])
grid on
ylabel("Napätie (V)")
subplot(3,1,2)
plot(t,Theta,LineWidth=1.5)
legend("Theta","Location","southeast")
xticks(0:0.25:1.5)
ylim([-0.2 1.7])
grid on
ylabel("Uhol (Rad)")
subplot(3,1,3)
plot(t,Alpha,LineWidth=1.5)
legend("Alpha","Location","southeast")
xticks(0:0.25:1.5)
ylim([-0.1 0.05])
grid on
ylabel("Uhol (Rad)")
xlabel ("Čas (s)")

%%
save('LinkShield_SSID.mat','model','Ts')