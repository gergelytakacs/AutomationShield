clear all; close all; clc;
%% 
readtable pid1.csv;                             
load BicopShield_GreyboxModel_LinearSS.mat;
down_const = 5;
y=downsample(ans.Var1,down_const);    
ref=zeros(3,length(y));
ref(1,:)=downsample(ans.Var2,down_const); 
ref=deg2rad(ref);
u1=downsample(ans.Var3,down_const);                                    % motor1 input
u2=downsample(ans.Var4,down_const);                                    % motor2 input
u=ans.Var5;

%% Model augmentation
Ts=0.01
intA = [model.A, zeros(2, 1); -model.C(1,:), ones(1,1)];
intB = [model.B; 0 0];
intC = [model.C(1,:), 0];
continuos_model=ss(intA,intB,intC,[0 0]);
zmodel = c2d(continuos_model,Ts,'foh')
%% LQ Gain and stability
Q_lqr=diag([10 1 1]);
R_lqr=diag([1e-2 1e-2]);
K=dlqr(zmodel.A,zmodel.B,Q_lqr,R_lqr);
body=eig(zmodel.A-zmodel.B*K)

% stability
num_points = 100;
theta = linspace(0, 2*pi, num_points);
x = cos(theta);
y = sin(theta);

plot(x, y);
hold on
plot(body,'x');
axis equal; 
xlabel('x');
ylabel('y');


%% Simulation
t=0:0.01:48-0.01;               %time vector
x_sim=zeros(3,length(t));
u_sim=zeros(2,length(t));
y_sim=zeros(1,length(t));
x1prev=0;

for i=1:length(t)-1  
u_lq=-K*(x_sim(:,i)-ref(:,i));  % computation
if u_lq(1,1)>30                 %saturation
    u_lq(1,1)=30;   
elseif u_lq(1,1)<-30
    u_lq(1,1)=-30;
end
if u_lq(2,1)>30
    u_lq(2,1)=30;
elseif u_lq(2,1)<-30
    u_lq(2,1)=-30;
end
x_sim(:,i+1)=zmodel.A*x_sim(:,i)+zmodel.B*u_lq;        % model simulation
x_sim(3,i+1)=x_sim(3,i)+Ts*(ref(1,i)-x_sim(1,i+1));    %integrator
u_sim(:,i)=u_lq;
end
%%
figure
subplot(3,1,1)
plot(t, x_sim(1,:),'LineWidth', 1.5);
hold on
plot(t,ref(1,:));
ylabel('Uhol [rad]');
legend('Measured data LQR','Reference')

subplot(3,1,2)
plot(t, x_sim(2,:),'LineWidth', 1.5);
ylabel('Angular speed [rad/s]');


subplot(3,1,3)
plot(t, u_sim(1,:),'LineWidth', 1.5);
hold on 
plot(t, u_sim(2,:),'LineWidth', 1.5);
ylabel('Percent [%]');
xlabel('Time [s]');
legend('Input motor 1','Input motor 2');
