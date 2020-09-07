load MagnetoShield_Models_Greybox_TF
load MagnetoShield_ID_Data

%% Response in closed-loop
P=c2d(model,Ts);                                % Plant, but discrete

% Original feedback controller used in the identification procedure
Kp=2.3;                                         % [V*mm]
Ti=0.1;                                         % [s]
Td=0.02;                                        % [s]
C  = pidstd(Kp,Ti,Td,inf,Ts)                    % Baseline controller
S  = feedback(C*P,-1);                           % Closed-loop negative feedback


figure(2)                                       % New figure
subplot(2,1,1)                                  % Subplot structure
Tsim=0:Ts:(length(y)-1)*Ts;                     % Time vector
U = u-u0;                                         % True input minus linearization point is delta input
Y = y0*1000-lsim(S,U,Tsim);                     % Simulated output plus original linearization point                            
plot(Tsim,Y)                                    % Simulated output
hold on                                         % Hold graph
plot(Tsim,y*1000)                               % Experiment output in mm
legend('Simulation','Experiment')               % Figure legend     
xlabel('Time (s)')                              % X-label
ylabel('Distance (mm)')                         % Y-label
grid on                                         % Grid on
axis([0,20,0,20])                               % Set axis          

subplot(2,1,2)                                  % Subplot structure     
plot(Tsim,u)                                    % Experiment input in V
legend('Experiment')                            % Figure legend     
xlabel('Time (s)')                              % X-label
ylabel('Voltage (V)')                           % Y-label
grid on                                         % Grid on
axis([0,20,1,12])                               % Set axis       