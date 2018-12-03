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
  
%clc; clear all;                              % Clears screen and all variables
% load resultID
Ts=0.004                                      % [s] Sampling
u=resultID(:,1)+resultID(:,2);
y=resultID(:,3)/1000;

data = iddata(y,u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid Voltage';            % Input name
data.InputUnit =  'V';                          % Input unit
data.OutputName = 'Position';                   % Output name
data.OutputUnit = 'm';                          % Output unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(241:end)                             % Discard the time when magnet is on ground, pick close to linearization point

data=detrend(data);
%data=idfilt(data,5,0.12732);    % Remove high frequency content

% Parameters
% m = 0.76E-3;                                    % [kg] Magnet mass
% R = 198.3;                                      % [Ohm] Solenoid resistance
% L = 0.239;                                      % [H] Solenoid inductance
% mu0 = 4*pi*1E-7;                                 % [H/m] Permeability of free space
% mur = 4000                                      % [-] Ratio for the permeability of the core vs. free space (~2000-6000)
% mu = mu0*mur;                                   % [H/m] Permeability of the core
% N =  2500;                                       % [-] Number of windings (turns)
% d = 8E-3;                                       % [m] Solenoid core diameter
% A = pi*(d/2)^2;                                 % [m^2] Solenoid core cross-sectional area

alpha=1000;
beta=3;
gamma=11;
delta=10;
h0=data(1).y;                                   % Initial position estimate
dh0=(data(2).y-data(1).y)/Ts;                   % Initial velocity estimate
i0=data(1).u/R;                                 % Initial current estimate

% Beta and alpha shouldbe positive, but then beta should be minus
A=[0       1    0;
   alpha   0   -beta
   0       0   -gamma];
B=[0;
   0;
   delta];
C=[1 0 0];
D=[0];
K = zeros(3,1);                   % Disturbance
x0=[h0; dh0; i0];

sys=idss(A,B,C,D,K,x0,0);
% compare(data,sys)
% axis([0.9600   19.9960   -0.0015    0.0015])

sys.Structure.A.Free=  [0     0    0;
                        1     0    1;
                        0     0    1];
sys.Structure.B.Free=  [0;
                        0;
                        1];                
sys.Structure.C.Free=  false;
sys.Structure.K.Free=  true;

% sys.Structure.A.Minimum=[-Inf -Inf -Inf;
%                           0   -Inf -Inf;
%                          -Inf -Inf -Inf  ];
% sys.Structure.A.Maximum=[ Inf  Inf  Inf;
%                           Inf  Inf  0;
%                           Inf  Inf  0  ];
%sys.Structure.B.Minimum=[0 0 0]';
sys.DisturbanceModel = 'estimate'
sys.InitialState = 'estimate'

Options = ssestOptions;                              
Options.Display = 'on';                              
Options.Focus = 'prediction';   
Options.WeightingFilter = [0 450];   
Options.EnforceStability = 1;                        
Options.InitialState = 'estimate';  
Options.SearchMethod='fmincon' %gn,gna,lm,grad,lsqnonlin,fmincon
Options.SearchOptions.MaxIterations=100;
Options.SearchOptions.StepTolerance=1E-10;
Options.SearchOptions.FunctionTolerance=1E-10;
model = ssest(data,sys,Options);
compare(data,model)
axis([0.9600   19.9960   -0.0015    0.0015])
