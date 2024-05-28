clear all; close all; clc;
%%

load BicopShield_GreyboxModel_LinearSS.mat      % Include linearized state-space model


filter = 'model';                           % Select state estimation method: 'Kalman', 'model', 'difference'
exportempc = 1;                             % Would you like to export this controller right away?
                                                               
                                                               
Ts=0.01;                                    % [s] sampling for discrete control
N=3;                                        % [steps] prediction horizon
ul=[-30; -30]                                       % [V] Adjust lower input constraint for linearization
uh=[30; 30]                                     % [V] Adjust upper input constraint for linearization

R=[0.8,1.0,0.35,1.3,1.0,0.35,0.8,1.2]'; % [rad] Reference levels for the simulation
T=1000;                                     % [samples] Section length for each reference level

t=0:Ts:(T*length(R)-1)*Ts;                  % [s] Time vector for the simulation
i=1;                                        % [-] Section counter for the simulation
r=R(1);                                     % [mm] First reference to start simulation
                   
%% Discretization
modeld=c2d(model,Ts);                      % Discretized linear state-space model
A=modeld.a;                                 % Extract A
B=modeld.b;                                 % Extract B
C=[1 0];                                    % C for introducing an integration component

Q_Kalman = diag([0.01, 10]);               % Process noise covariance matrix 
R_Kalman = diag([1e0]);             % Measurement noise covariance matrix

                                 
%% Model augmentation by integration
[ny, ~]=size(C);                            % Sizing C
[nx, nu]=size(B);                           % Sizing B
Ai=[eye(ny,ny) -C;                          % Augmenting A by an integrator
    zeros(nx,ny)  A];          
Bi=[zeros(ny,nu); B];                       % Augmenting B by the integrator
Ci=[zeros(ny,ny) C];                        % Augmenting C by the integrator

%% MPC penalty
Qmpc=diag([1 10 1]);                                     % State penalty matrix
Rmpc=diag([1e-2, 1e-2]);                                   % Input penalty matrix

%% Model and EMPC controller using the MPT 3.0
modelEMPC = LTISystem('A', Ai, 'B', Bi, 'C', Ci, 'Ts', Ts); % LTI model by the MPT 3.0
modelEMPC.u.min = ul;                                       % [V] Lower input constraint for the MPT
modelEMPC.u.max = uh;                                       % [V] Upper input constraint for the MPT
modelEMPC.x.penalty = QuadFunction(Qmpc);                   % State penalty for the MPT
modelEMPC.u.penalty = QuadFunction(Rmpc);                   % Input penalty for the MPT
PA = modelEMPC.LQRPenalty;                                  % Compute LQ penalty by the MPT
modelEMPC.x.with('terminalPenalty');                        % Enable terminal penalty in the MPT
modelEMPC.x.terminalPenalty = PA;                           % Define for the MPT
% Compute controller
ctrl = MPCController(modelEMPC, N)                          % Create an MPC controller by the MPT
ectrl = ctrl.toExplicit();                              % Transform it to EMPC by the MPT

%% Simulation
U=[0;0]
x0=[0 0]';                                % Initial condition
X=x0;                                     % State logging (initialization)
xI=0;                                     % Integrator initialization
y=[0]';                                   % Output logging (initialization)
Xhat=[0;0];
x1p=0;                                    % Previous state
xICWhole = [0;0];                         % Initial conditions for Kalman filter

xhat = zeros(2,1);                        % States estimated by Kalman filter
yhat = zeros(1,1);                        % Outputs estimated by Kalman filter
Y = zeros(1,1);
kalmanrychlost=zeros(1,length(measuredOutput));
disp('Simulating...')
for k=1:length(t)-1
    Rr(k)=R(i);                           % Original reference, just for logging
    % Experiment (simulation) profile
    if (mod(k,(T*i)) == 0)                % At each section change
        i = i + 1;                        % we move to the next index
        r = R(i);                         % and use the actual reference
    end
    
    % Measurement, primitively reconstructing system states from "measurements"     
    x1=X(1,k);                                                  
    x2=X(2,k);                                                  
    Y(1,1) = x1;                          % Y is used to pass variable x1 to estimateKalmanState function

    
%State estimation based on user option   
if strcmp(filter,'Kalman')
  % Ukal(:,k)=inputFull(k,:);
  % Y=measuredOutput(k);
       [xhat, yhat] = estimateKalmanState(U(:,k), Y, A, B, C, Q_Kalman, R_Kalman, xICWhole); 
       kalmanrychlost(k)=yhat(1,1);
elseif strcmp(filter,'model')
       xhat = [X(1,k); X(2,k)];
elseif strcmp(filter,'difference')
       x2 = (x1 - x1p)/Ts;
       x1p=x1; 
       xhat = [x1; x2];
end
    % Control
    xI(k+1)=xI(k)+(r-x1);                 % Integrator state
    U(:,k+1)=ectrl.evaluate([xI(k); xhat]); % EMPC compensated by the linearization point
    
    % Model
    X(:,k+1)=A*X(:,k)+B*(U(:,k+1));         % System model
%     X(1,k+1)=constrain(X(1,k+1),-pi,pi);  % angle limits
    y(1,k+1)=X(1,k);                      % Position measurement on the real system
    Xhat(:,k+1)=xhat;                     % Estimated state vector for logging
end
disp('...done.')

%% Plotting

figure(1);                                % New figure

subplot(2,1,1)                            % Top subplot
plot(t(1:end-1),Rr);                      % Plot reference
hold on                                   % Draw on top of this
plot(t(1:end-1),y(1,1:end-1),'LineWidth',1.5);            % Plot the output (position)
grid on                                   % Grid is enabled
xlabel('Time [s]')                        % X axis label
ylabel('Angle [rad]')                  % Y axis label
legend('Reference','Measured angle')
title('Simulation - EMPC')

subplot(2,1,2)                            % Bottom subplot
plot(t(1:end),U,'LineWidth',1.5);                         % Plot inputs
grid on                                   % Allow grid
xlabel('Time [s]')                        % X axis label
ylabel('Input [%]')                       % Y axis label
legend('Input motor 1','Input motor 2')

% if kalman filter is used print comparison between system state and estimated state
if strcmp(filter,'Kalman')
    figure(2);

    subplot(2,1,1)                        % Top subplot
    plot(t,X(1,:),'LineWidth',1.5);                       % Plot reference
    hold on                               % Draw on top of this
    plot(t,Xhat(1,:),'LineWidth',1.5);                    % Plot the output (position)
    hold off
    grid on                               % Grid is enabled
    xlabel('Time [s]')                    % X axis label
    ylabel('Anlge [rad]')              % Y axis label
    legend('State 1 - Kalman filter','State 1 - sim without filter')
    
    subplot(2,1,2)                        % Bottom subplot
    plot(t,X(2,:),'LineWidth',1.5);                       % Plot reference
    hold on                               % Draw on top of this
    plot(t,Xhat(2,:),'LineWidth',1.5);                    % Plot the output (position)
    hold off
    grid on                               % Grid is enabled
    legend('State 2 with Kalman filter','State 2 - without Kalman filter')
    xlabel('Time [s]')                    % X axis label
    ylabel('Angular speed [rad/s]')    % Y axis label


%     % Print matrices in specific format to be used in Arduino example
    disp(' ')
    disp('Discrete SS Matrices: ')
    printSSMatrix(A, 'A')
    printSSMatrix(B, 'B')
    printSSMatrix(C, 'C')
    printSSMatrix(Q_Kalman, 'Q_Kalman')
    printSSMatrix(R_Kalman, 'R_Kalman')
end

%% export controller files for usage in arduino example and Python
if exportempc
    ectrl.exportToC('ectrl','cmpc')       % Export to C
    empcToC(ectrl,'AVR');                 % Export to code, for ARM MCU use 'generic', for AVR use 'AVR'
    % empcToPython(ectrl);                  % Export to code suitable for Python
end


