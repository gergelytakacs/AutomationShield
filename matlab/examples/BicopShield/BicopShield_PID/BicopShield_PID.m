clear all;close all; clc
%%
%open 
readtable   pid1.csv;
y=ans.Var1;        % angle
ref=ans.Var2;      % reference
u1=ans.Var3;       % motor1
u2=ans.Var4;       % motor2
u=ans.Var5;        % PID output

Ts=0.01;            % Sampling period
t=0:Ts:239.98;      % Time vector
t=t';   

% Measured data plot
figure(1)
plot(t,y, 'LineWidth', 1.5);
hold on
grid on
plot(t,ref);
legend('Namerané dáta','Žiadaná hodnota');
title('Namerané dáta');
xlabel('Čas (s)');
ylabel('Uhol ( °)');


% Identification ARX
% parameters for arx
na=4;
nb=4;
nk=1;
data=iddata(y,u,'Ts',Ts);   % Data object
sys=arx(data,[na nb nk]);   % Identification

sim_data=sim(sys,u);        % Simulation

figure(3)
plot(sim_data,'LineWidth', 1.5)
grid on
legend('Identificated ARX model');
title('ARX Model');
xlabel('Time (s)');
ylabel('Angle ( °)');

%compare model a validačné data
figure(4)
compare(sys,data,100)
grid on
legend('Validation data','Identificated ARX model');
xlabel('Time (s)');
ylabel('Angle ( °)');

% PID controller
Kp=1.5;                       
Ki=2.0;                       
Kd=0.5;
pid_controller=pid(Kp,1/Ki,Kd,0,Ts);
sys_PID=feedback(pid_controller*sys,1); 

% Simulation with PID
[y_sim, t_sim] = lsim(sys_PID, ref, t);

% Plot Results
figure(5)
plot(t,ref,'b');
grid on
xlabel('Time (s)');
ylabel('Angle ( °)');
title('ARX Model with PID');
hold on
plot(t,y,'LineWidth', 1.5)
plot(t_sim, y_sim,'LineWidth', 1.5);
legend('Reference','Measured data','ARX model with PID');





