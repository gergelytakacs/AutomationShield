%   MagnetoShield grey-box linearized state space system
%   identification example
% 
%   This example takes a measurement from the 
%   MagnetoShield device, where the permanent magnet is
%   levitatated around the 15 mm setpoint in closed-loop
%   using PID feedback. The experiment contains the 
%   input voltage, and two outputs: position and current.
%   The identification assumes that the dynamics can be 
%   represented using third order state-space model, 
%   that is based on the second order mechanical 
%    equation and the first order electrical
%   equation. Most literature asssumes that the 
%   inductance L of the solenoid is fixed 
%   (fixedInductance=1), here you also may choose to
%   model it as a distance-dependent variable 
%   (fixedInductance=0). The resulting model will be in
%   "delta" form, that is all variables relate to 
%   linearization point.

%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last updated by: Jakub Mihalík
%   Last update: 12.10.2020.
  
startScript;                                    % Clears screen and variables, except allows CI testing

fixedInductance=0;                              % Fixed or distance dependent inductance?
                                                % to mathematical model
load MagnetoShield_ID_Data.mat                  % Load data file
Ts=0.003250;                                    % [s] Sampling
y=result(:,1)/1000;                             % [m] Output in meters
u=result(:,2);                                  % [V] Input is closed loop + probe signal
i=result(:,3)/1000;                             % [A] Current

%% Parameters
g  = 9.81;										% [m/s2] Gravitational acceleration		  
m  = 0.945E-3;                                  % [kg] Magnet mass
R  = 198.3;                                     % [Ohm] Solenoid resistance
L  = 0.239;                                     % [H] Solenoid inductance
Km = 3.333e-06;                     % Magnetic constant for the mechanical part
Ke = Km;                                        % Magnetic constant for the electrical part

% Linearization points based on the experiment
y0 = mean(y);                                   % [m] Output (position) linearization around setpoint
u0 = mean(u);                                   % [V] Input linearization around setpoint
i0 = mean(i);                                   % [A] Current linearization around setpoint

% Exact linearization points based on the model
% i0 = sqrt(g*m*y0^2/Km);                        % [A] Current linearization around setpoint
% u0 = R*i0;                                     % [V] Input linearization around setpoint

delta_y = y-y0;                                 % [m] Distance data adjusted by linearization points
delta_i = i-i0;                                 % [A] Current data adjusted by linearization points
delta_u = u-u0;                                 % [V] Voltage input data adjusted by linearization points

% Initial parameters for generic linearized model
alpha   = 2*(Km/m)*i0^2/y0^3;                   % Linearized parameter guess
beta    = 2*(Km/m)*i0/y0^2;                     % Linearized parameter guess
gamma   = 2*Ke*i0/(L*y0^2);                     % Linearized parameter guess
delta   = R/L;                                  % Parameter guess
epsilon = 1/L;                                  % Parameter gues

%% System identification data object
data = iddata([delta_y delta_i],delta_u,Ts,'Name','Magnetic Levitation');    % Data file
data.InputName = 'Solenoid';                    % Input name: Solenoid voltage
data.InputUnit =  'V';                          % Input unit
data.OutputName{1} = 'Position';                % Output 1 name
data.OutputUnit{1} = 'm';                       % Output 1 unit
data.OutputName{2} = 'Current';                 % Output 2 name
data.OutputUnit{2} = 'A';                       % Output 2 unit
data.Tstart = 0;                                % Starting time
data.TimeUnit = 's';                            % Time unit
data = data(100:end);                           % Discard the time when magnet is on ground, pick close to linearization point
data=detrend(data);                             % Discard offset, takes care of linearization too (if mean is taken as lin. point)
dataf = fft(data);                              % Frequency domain 																		   

% Initial states
h0=data.y(1,1);                                 % Initial position estimate
dh0=(data.y(2,1)-data.y(1,1))/Ts;               % Initial velocity estimate
ii0=data.y(1,2);                                 % Initial current estimate

%% Construct model
if fixedInductance==1                           % Magnet inductance L fixed
    A=[0       1    0;
       alpha   0   -beta
       0       0   -delta];

elseif  fixedInductance==0                      % Magnet inductance L(y) distance dependent 

    A=[0       1        0;                      % Dybamic matrix initial guess
       alpha   0       -beta
       0      -gamma   -delta];     
end

B=[0; 0; epsilon];                              % Input matrix
C=[1 0 0;                                       % Output matrix
   0 0 1];                                      % Distance and current measured
D=[0; 0];                                       % No feed-through                                            
K = zeros(3,2);                                 % Disturbance
x0=[h0; dh0; ii0];                               % Initial condition
disp('Initial guess:')
sys=idss(A,B,C,D,K,x0,0)                        % Construct state-space representation


% Mark the free parameters
if fixedInductance==1                           % Magnet inductance L fixed
    sys.Structure.A.Free=      [0     0    0;   % Free and fixed variables
                                1     0    1;
                                0     0    1];
elseif fixedInductance==0                       % Magnet inductance L(y) distance dependent
    sys.Structure.A.Free=      [0     0    0;   % Free and fixed variables
                                1     0    1;
                                0     1    1];
end
                      
sys.Structure.B.Free=  [0  0 1]';               % Free and fixed variables
sys.Structure.C.Free=  false;                   % No free parameters

sys.DisturbanceModel = 'estimate';              % Estimate disturbance model
sys.InitialState = 'estimate';                  % Estimate initial states

%% Set estimation options
Options = ssestOptions;                         % State-space estimation options
Options.Display = 'on';                         % Show progress
Options.Focus = 'simulation';                   % Identification focus
Options.EnforceStability = 0;                   % Unstable model
Options.InitialState = 'estimate';              % Estimate initial condition

%% Estimate and list parameters
disp('Identified model:')
model = ssest(dataf,sys,Options)                % Launch estimation procedure
compare(dataf,model,1)                            % Compare data to model

disp('---Parameter Comparison---')
Ld=['L (measured): ',num2str(L),' [H],   L (model): ',num2str(1/model.b(3),3),' H'];
disp(Ld)
Rd=['R (measured): ',num2str(R),' [Ohm], R (model): ',num2str(-model.a(3,3)/model.b(3),3),' [Ohm]'];
disp(Rd)

if fixedInductance==0
 Kmodel=([model.a(2,1)*m*y0^3/(2*i0^2) -(model.a(3,2)*L*y0^2)/(2*i0)]);
 Kstd=std([model.a(2,1)*m*y0^3/(2*i0^2) -(model.a(3,2)*L*y0^2)/(2*i0)]);
 Kdm=['Km (guess): ',num2str(Km), ',    Km (model): ',num2str(Kmodel(1))];
 disp(Kdm)
 Kde=['Ke (guess): ',num2str(Km), ',    Ke (model): ',num2str(Kmodel(2))];
 disp(Kde)
 end


save('MagnetoShield_Models_Greybox_SS','model','y0','u0','i0','Ts')      % Save plant model (mm)