%   HeatShield system identification example
% 
%   Measures and logs the printer head temperature for
%   system identification purposes.
%
%   This example initializes the HeatShield device, then
%   supplies a step change of input to 40% power to the
%   heating cartridge and plots the response. When the
%   experiment is over, the data is saved to a file.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 18.11.2018.
  
clc; clear all;                               % Clears screen and all variables
load resultID
Ts=0.004                                      % [s] Sampling
u=resultID(:,1)+resultID(:,2);
y=resultID(:,3)/1000;

fixedInductance=0;                         % Fixed or distance dependent inductance?

data = iddata(y,u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid Voltage';            % Input name
data.InputUnit =  'V';                          % Input unit
data.OutputName = 'Position';                   % Output name
data.OutputUnit = 'm';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(241:end)                            % Discard the time when magnet is on ground, pick close to linearization point
data=detrend(data);                             % Discard offset
dataf = fft(data);                              % Frequency domain 

%% Parameters
m = 0.76E-3;                                    % [kg] Magnet mass
R = 198.3;                                      % [Ohm] Solenoid resistance
L = 0.239;                                      % [H] Solenoid inductance
u0=mean(u);                                     % [V] Input linearization around setpoint
y0=mean(y);                                     % [m] Output linearization around setpoint
if fixedInductance==1
K=4.5E-6;                                         % Magnetic constant (rhough estimate)
elseif  fixedInductance==0
K=4.5E-6;   
end
    
% Initial parameters for linearized model
h0=data(1).y;                                   % Initial position estimate
dh0=(data(2).y-data(1).y)/Ts;                   % Initial velocity estimate
i0=-data(1).u/R;                                 % Initial current estimate
alpha=2*(K/m)*u0^2/y0^3;                        % Linearized parameter guess
beta=2*(K/m)*u0/y0^2;                           % Linearized parameter guess

%% Construct model
if fixedInductance==1
A=[0       1    0;
   alpha   0   -beta
   0       0   -R/L];

elseif  fixedInductance==0
i0=-u0/R;    %<==== This must be improved.
gamma=2*(K/m)*i0/(L*y0^2);
A=[0       1        0;
   alpha   0       -beta
   0       gamma   -R/L];     
end

B=[0;
   0;
   1/L];
C=[1 0 0];
D=[0];
K = zeros(3,1);                                 % Disturbance
x0=[h0; dh0; i0];                               % Initial condition
sys=idss(A,B,C,D,K,x0,0);

if fixedInductance==1
sys.Structure.A.Free=  [0     0    0;
                        1     0    1;
                        0     0    1];
sys.Structure.A.Minimum=[-Inf -Inf -Inf;
                          0   -Inf -Inf;
                         -Inf -Inf -Inf  ];
sys.Structure.A.Maximum=[ Inf  Inf  Inf;
                          Inf  Inf  0;
                          Inf  Inf  0  ]; 
elseif fixedInductance==0
sys.Structure.A.Free=  [0     0    0;
                        1     0    1;
                        0     1    1];
sys.Structure.A.Minimum=[-Inf -Inf -Inf;
                          0   -Inf -Inf;
                         -Inf  -Inf -Inf  ];
sys.Structure.A.Maximum=[ Inf  Inf  Inf;
                          Inf  Inf  0;
                          Inf  Inf  0  ]; 
end
                      
                    
sys.Structure.B.Free=  [0;
                        0;
                        1];                
sys.Structure.B.Minimum=[-Inf -Inf 0]';
sys.Structure.B.Maximum=[Inf Inf Inf]';
sys.Structure.C.Free=  false;
sys.Structure.K.Free=  true;
sys.DisturbanceModel = 'estimate';
sys.InitialState = 'estimate';

%% Set estimation options
Options = ssestOptions;                              
%Options.Display = 'on';                              
Options.Focus = 'simulation';   
Options.EnforceStability = 0;                        
Options.InitialState = 'estimate';  
Options.SearchMethod='auto' %gn,gna,lm,grad,lsqnonlin,fmincon
Options.SearchOptions.MaxIterations=100;
%Options.SearchOptions.StepTolerance=1E-10;
%Options.SearchOptions.FunctionTolerance=1E-10;
model = ssest(dataf,sys,Options)
compare(dataf,model)

save MagnetoShield_Models_Greybox_SS model     % Save model

%% Parameter information
disp('---Parameter Comparison---')
Ld=['L (measured): ',num2str(L),' [H],   L (model): ',num2str(1/model.b(3),3),' H'];
disp(Ld)
Rd=['R (measured): ',num2str(R),' [Ohm], R (model): ',num2str(-model.a(3,3)/model.b(3),3),' [Ohm]'];
disp(Rd)
