%   MAGNETOSHIELD LQ GAIN CALCULATION AND SIMULATION EXAMPLE
%
%   This example reads the linearized model of the MagnetoShield device,
%   then expands it with an integrator and then computes the LQ gain. This
%   LQ gain is used to crate a simulation run for the trajectory tracking
%   of magnetic levitation. The user may switch between direct state
%   measurement, numerical differentiation and a Kalman filter. Disturbance
%   may be added to the signal to make it more realistic. The Kalman filter
%   matrices and the LQ gain are exported into the BLA matrix format, that
%   is used in the actual MagnetoShield LQ example, see MagnetoShield_LQ.ino.
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
%   Created by:       Gergely Takács and Gábor penzinger. 
%   Created on:       23.9.2020
%   Last updated by:  Gergely Takács
%   Last update on:   24.9.2020


clc; clear; close all;                                          % Close and clear all

load MagnetoShield_Models_Greybox_SS                            % Include linearized state-space model

disturbance=1;                                                  % Adds disturbance to simulation (1)

filter = 'Kalman';                                              %Select state estimation method: 'Kalman', 'model', 'difference'

                                                               
Ts=0.004;                                                       % [s] sampling for discrete control
R=[14.0,13.0,15.0,14.5,13.5,13.0]';                             % [mm] Reference levels for the simulation
T=1000;                                                         % [samples] Section length for each reference level

y0= 14.3;                                                       % [mm] Linearization point based on the experimental identification
i0= 0.0219;                                                     % [A] Linearization point based on the experimental identification
u0= 4.6234;                                                     % [V] Linearization point based on the experimental identification

ra=(R-y0)/1000;                                                 % [mm] Adjusted reference levels for the linearization point
t=0:Ts:(T*length(R)-1)*Ts;                                      % [s] Time vector for the simulation
umin=0;                                                         % [V] Lower input constraint
umax=12;                                                        % [V] Upper input constraint
i=1;                                                            % [-] Section counter for the simulation
r=ra(1);                                                        % [mm] First reference to start simulation
                   
%% Discretization
modeld=c2d(model,Ts);                                           % Discretized linear state-space model
A=modeld.a;                                                     % Extract A
B=modeld.b;                                                     % Extract B
C=[1 0 0];                                                      % C for introducing an integration component


if disturbance
    Q_Kalman = diag([0.0001, 100, 100]);                          %process noise  covariance matrix 
    R_Kalman = diag([0.001, 0.001]);                              %measurement noise covariance matrix
else
    Q_Kalman = diag([0.0001, 100, 10]);
    R_Kalman = diag([0.001, 0.001]);
end

                                   


%% Model augmentation by integration
[ny nx]=size(C);                                                % Sizing C
[nx nu]=size(B);                                                % Sizing B
Ai=[eye(ny,ny) -C;                                              % Augmenting A by an integrator
    zeros(nx,ny)  A];          
Bi=[zeros(ny,nu); B];                                           % Augmenting B by the integrator
Ci=[zeros(ny,nx) C];                                            % Augmenting C by the integrator

%% LQ design
Qlq=diag([50 100 100 10]);                                     % State penalty matrix
Rlq=0.1;                                                       % Input penalty matrix
K=dlqr(Ai,Bi,Qlq,Rlq);                                          % LQ gain

%% Simulation
U=0;
x0=[1E-3 0 0]';                                                 % Initial condition
X=x0;                                                           % State logging (initialization)
xI=0;                                                           % Integrator initialization
y=[17 0]';                                                      % Output logging (initialization)
x1p=0;                                                          % Previous state
xICWhole = [0;0;0.0223];                                        % Initial conditions for Kalman filter

xhat = zeros(3,1);                                              %States estimated by Kalman filter
yhat = zeros(2,1);                                              %Outputs estimated by Kalman filter
Y = zeros(1,2);
for k=1:length(t)-1
    % Experiment (simulation) profile
    Rr(k)=R(i);                                                 % Original reference, just for logging
    if (mod(k,(T*i)) == 0)                                      % At each section change
        i = i + 1;                                              % We move to the next index
        r = ra(i);                                              % and use the actual, corrected reference
    end
    
    % Measurement, primitively reconstructing system states from "measurements"     
    x1=y(1,k)/1000-y0/1000;                                     % Compensate for meters and linearization point from direct measurement
    x2=X(2,k);                                                  % Previous position output (state estimate)
    x3=y(2,k)/1000-i0;                                          % Compensate for Amperes and linearization point from direct measurement
    Y(1,1) = x1;                                                %Y is used to pass variables x1 and x2 to estimateKalmanState function
    Y(1,2) = x3;
    
%State estimation based on user option   
if strcmp(filter,'Kalman')
       [xhat, yhat] = estimateKalmanState(U(k), Y', A, B, modeld.c, Q_Kalman, R_Kalman, xICWhole); 
elseif strcmp(filter,'model')
       xhat = [x1; x2; x3];
elseif strcmp(filter,'difference')
       x2 = (x1 - x1p)/Ts;
       x1p=x1; 
       xhat = [x1; x2; x3];
end
    % Control
    xI(k+1)=xI(k)+(r-x1);                                       % Integrator state
    U(k+1)=-K*[xI(k); xhat]+u0;                                 % LQ compensation for the integrator augmented linear part
    U(k+1)=constrain(U(k+1),umin,umax);                         % Constrain inputs like on the real system
    
    % Model
    X(:,k+1)=A*X(:,k)+B*(U(k+1)-u0);                            % System model
    X(1,k+1)=constrain(X(1,k+1),12E-3-y0*1E-3,17E-3-y0*1E-3);   % Cage limits
    X(3,k+1)=constrain(X(3,k+1),-i0,60E-3-i0);                  % No reverse current possible
    if disturbance
        X(1,k+1)=X(1,k+1)+rand(1)*0.3E-4;                       % Add random position disturbance (e.g. process noise)
    end
    y(1,k+1)=X(1,k)*1000+y0;                                    % Position measurement on the real system
    y(2,k+1)=(X(3,k)+i0)*1000;                                  % Current measurement on the real system
  
end

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

subplot(2,1,2)                                                  % Bottom subplot
plot(t(1:end),U);                                               % Plot inputs
grid on                                                         % Allow grid
xlabel('Time (s)')                                              % X axis label
ylabel('Input (V)')                                             % Y axis label


%% Print matrices in specific format to be used in Arduino example
disp(' ')
disp('Discrete SS Matrices: ')
printSSMatrix(modeld.a, 'A')
printSSMatrix(modeld.b, 'B')
printSSMatrix(modeld.c, 'C')
printSSMatrix(Q_Kalman, 'Q_Kalman')
printSSMatrix(R_Kalman, 'R_Kalman')
printSSMatrix(K, 'K')




