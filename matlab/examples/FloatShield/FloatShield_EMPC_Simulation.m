%   AEROSHIELD EXPLICIT MPC SIMULATION EXAMPLE
%
%   This example reads the linearized model of the AeroShield device,
%   then expands it with an integrator and then computes an explicit model
%   predictive controller (EMPC, Explicit MPC) using the Multi-Parametric
%   Toolbox of Kvasnica et al. The code has been tested with v. 3.0 of the
%   MPT. Please see https://www.mpt3.org/ for the free download and
%   installation instruction. 
%   The EMPC controller is used to create a simulation run for the trajectory 
%   tracking of aero pendulum. The user may switch between direct state
%   measurement, numerical differentiation and a Kalman filter. 
%   The Kalman filter matrices and are exported into the BLA matrix format, that
%   is used in the actual AeroShield LQ example, see AeroShield_EMPC.ino.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   If you have found any use of this code, please cite our work in your
%   academic publications, such as thesis, conference articles or journal
%   papers. A list of publications connected to the AutomationShield
%   project is available at: 
%   https://github.com/gergelytakacs/AutomationShield/wiki/Publications
%
%   Created by:       Gergely Takács and Erik Mikuláš
%   Created on:       9.8.2023
%   Last updated by:  Erik Mikuláš
%   Last update on:   9.8.2023


startScript;                                                    % Clears screen and variables, except allows CI testing

load FloatShield_GreyboxModel_LinearSS                          % Include linearized state-space model

filter = 'model';                                               % Select state estimation method: 'Kalman', 'model'
exportempc = 0;                                                 % Would you like to export this controller right away?
                                                               
                                                               
Ts=0.025;                                                       % [s] sampling for discrete control
N=3;                                                            % [steps] prediction horizon
ul=0.0;                                                        % [%] Adjust lower input constraint for linearization
uh=100.0;                                                       % [%] Adjust upper input constraint for linearization

R=[210.0, 160.0, 110.0, 145.0, 195.0, 245.0, 180.0, 130.0, 65.0,  95.0]';         % [mm] Reference levels for the simulation
T=1000;                                                         % [samples] Section length for each reference level

t=0:Ts:(T*length(R)-1)*Ts;                                      % [s] Time vector for the simulation
i=1;                                                            % [-] Section counter for the simulation
r=R(1);                                                         % [mm] First reference to start simulation
                   
%% Discretization
modeld=c2d(model,Ts);                                           % Discretized linear state-space model
A=modeld.a;                                                     % Extract A
B=modeld.b;                                                     % Extract B
C=[1 0 0];                                                      % C for introducing an integration component

Q_Kalman = diag([0.0001, 100, 100]);                            % process noise  covariance matrix 
R_Kalman = diag([0.001, 0.001]);                                % measurement noise covariance matrix

                                 
%% Model augmentation by integration
[ny, ~]=size(C);                                                % Sizing C
[nx, nu]=size(B);                                               % Sizing B
Ai=[eye(ny,ny) -C;                                              % Augmenting A by an integrator
    zeros(nx,ny)  A];          
Bi=[zeros(ny,nu); B];                                           % Augmenting B by the integrator
Ci=[zeros(ny,ny) C];                                            % Augmenting C by the integrator

%% MPC penalty
Qmpc=diag([10 5 10 100]);                                           % State penalty matrix
Rmpc=2e2;                                                       % Input penalty matrix

%% Model and EMPC controller using the MPT 3.0
model = LTISystem('A', Ai, 'B', Bi, 'C', Ci, 'Ts', Ts);         % LTI model by the MPT 3.0
model.u.min = ul;                                               % [V] Lower input constraint for the MPT
model.u.max = uh;                                               % [V] Upper input constraint for the MPT
model.x.penalty = QuadFunction(Qmpc);                           % state penalty for the MPT
model.u.penalty = QuadFunction(Rmpc);                           % input penalty for the MPT
PA = model.LQRPenalty;                                          % compute LQ penalty by the MPT
model.x.with('terminalPenalty');                                % enable terminal penalty in the MPT
model.x.terminalPenalty = PA;                                   % define for the MPT

% Compute controller
ctrl = MPCController(model, N)                                  % create an MPC controller by the MPT
ectrl = ctrl.toExplicit();                                      % transform it to EMPC by the MPT

%% Simulation
U=0;
x0=[0 0 0]';                                                      % Initial condition
X=x0;                                                           % State logging (initialization)
xI=0;                                                           % Integrator initialization
y=[0]';                                                         % Output logging (initialization)
x1p=0;                                                          % Previous state
xICWhole = [0;0;0];                                               % Initial conditions for Kalman filter

xhat = zeros(3,1);                                              %States estimated by Kalman filter
yhat = zeros(1,1);                                              %Outputs estimated by Kalman filter
Y = zeros(1,1);
disp('Simulating...')
for k=1:length(t)-1
    Rr(k)=R(i);                                                 % Original reference, just for logging
    % Experiment (simulation) profile
    if (mod(k,(T*i)) == 0)                                      % At each section change
        i = i + 1;                                              % We move to the next index
        r = R(i);                                               % and use the actual reference
    end
    
    % Measurement, primitively reconstructing system states from "measurements"                                                     
    Y(1,1) = X(1,k);                                            %Y is used to pass variable x1 to estimateKalmanState function

    
%State estimation based on user option   
if strcmp(filter,'Kalman')
       [xhat, yhat] = estimateKalmanState(U(k), Y', A, B, modeld.c, Q_Kalman, R_Kalman, xICWhole); 
elseif strcmp(filter,'model')
       xhat = [X(1,k); X(2,k); X(3,k)];
end

    % Control
    xI(k+1)=xI(k)+(r-xhat(1));                                    % Integrator state
    U(k+1)=ectrl.evaluate([xI(k+1); xhat]);                    % EMPC compensated by the linearization point
    
    % Model
    X(:,k+1)=A*X(:,k)+B*(U(k+1));                            % System model
    X(1,k+1)=constrain(X(1,k+1),0,300);                      % distance limits
    y(1,k+1)=X(1,k);                                         % Position measurement on the real system
end
disp('...done.')

%% Plotting

figure(1);                                                      % New figure

subplot(2,1,1)                                                  % Top subplot
plot(t(1:end-1),Rr);                                            % Plot reference
hold on                                                         % Draw on top of this
plot(t(1:end-1),y(1,1:end-1));                                  % Plot the output (position)
grid on                                                         % Grid is enabled
xlabel('Time (s)')                                              % X axis label
ylabel('Position (mm)')                                         % Y axis label
%legend('Reference','Position')
hold off

subplot(2,1,2)                                                  % Bottom subplot
plot(t(1:end),U);                                               % Plot inputs
grid on                                                         % Allow grid
xlabel('Time (s)')                                              % X axis label
ylabel('Input (%)')                                             % Y axis label


%% Print matrices in specific format to be used in Arduino example
disp(' ')
disp('Discrete SS Matrices: ')
printSSMatrix(modeld.a, 'A')
printSSMatrix(modeld.b, 'B')
printSSMatrix(modeld.c, 'C')
printSSMatrix(Q_Kalman, 'Q_Kalman')
printSSMatrix(R_Kalman, 'R_Kalman')

if exportempc
    ectrl.exportToC('ectrl','cmpc')                              % Export to C
    empcToC(ectrl,'generic');                                        % Export to code, for ARM MCU use 'generic', for AVR use 'AVR'
    empcToPython(ectrl);                                         % Export to code suitable for Python
end