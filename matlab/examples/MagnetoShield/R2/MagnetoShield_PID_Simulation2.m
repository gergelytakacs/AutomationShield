clc; clear; close all;                                          % Close and clear all

load MagnetoShield_Models_Greybox_TF                            % Include linearized state-space model

disturbance=1;                                                  % Adds disturbance to simulation (1)
disturbance_amp=1E-5;                                           % Process noise amplitude

Ts=5E-3;                                                        % [s] sampling for discrete control
R=[14.0,13.0,14.0,14.5,13.5,13.0]';                             % [mm] Reference levels for the simulation
T=100;                                                         % [samples] Section length for each reference level

y0=14.0068;                                                     % [mm] Linearization point based on the experimental identification
i0=0.0169;                                                      % [A] Linearization point based on the experimental identification
u0=3.6651 ;                                                     % [V] Linearization point based on the experimental identification

ra=(R-y0)/1000;                                                 % [mm] Adjusted reference levels for the linearization point
t=0:Ts:(T*length(R)-1)*Ts;                                      % [s] Time vector for the simulation
umin=0;                                                         % [V] Lower input constraint
umax=10;                                                        % [V] Upper input constraint
i=1;                                                            % [-] Section counter for the simulation
r=ra(1);                                                        % [mm] First reference to start simulation

P=c2d(model,Ts);                                                % Plant, but discrete
PD = idpoly(P,'NoiseVariance',0);                               % Plant as a difference equation
%% PID Feedback Controller Design

PID = PID;                        % Create PID object from PID class

Kp=5.1;                                                         % [V*mm]
Ti=0.1;                                                         % [s]
Td=0.03;                                                        % [s]
PID.setParameters(Kp, Ti, Td, Ts);                              % Feed the constants to PID object 

%% Simulation


y =17;                                                           % Output logging (initialization)
                                                                  % Previous state

Y = 0;
for k=1:length(t)-1
    % Experiment (simulation) profile
    Rr(k)=R(i);                                                 % Original reference, just for logging
    if (mod(k,(T*i)) == 0)                                      % At each section change
        i = i + 1;                                              % We move to the next index
        r = ra(i);                                              % and use the actual, corrected reference
    end

    U(k)   = PID.compute(r-Y(k), 0, 10, 0, 10);                % Compute PID response;
    Y(k+1) = diffeq(PD.f,PD.b,U(k));                              % System model
 

%     if disturbance
%         X(:,k+1)=X(:,k+1)+rand(1)*disturbance_amp;                         % Add random position disturbance (e.g. process noise)
%     end
  
end

plot(Y)
return

%% Plotting

figure(1);                                                      % New figure

subplot(2,1,1)                                                  % Top subplot
plot(t(1:end-1),Rr);                                            % Plot reference
hold on                                                         % Draw on top of this
plot(t(1:end-1),Y);                                  % Plot the output (position)
grid on                                                         % Grid is enabled
xlabel('Time (s)')                                              % X axis label
ylabel('Position (mm)')                                         % Y axis label
legend('Reference','Simulation')
axis([0,30,12,15])
subplot(2,1,2)                                                  % Bottom subplot
plot(t(1:end-1),U);                                             % Plot inputs
grid on                                                         % Allow grid
xlabel('Time (s)')                                              % X axis label
ylabel('Input (V)')                                             % Y axis label





