%   MotoShield Kalman State Estimate based on State Space Model.
% 
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Ján Boldocký.

startScript;

load identification_data.mat                             % Read identification results
load MotoShield_LinearSS.mat  
fs=1e5;
[ifltr,lp_filter]=lowpass(i,0.05,fs,'StopbandAttenuation',200,'Steepness',0.85);

Ts = 0.02;                                     % Sampling period
y=y/Ts*2*pi/28;         %_PPS to rad/sec
%%
data = iddata([y i],u,Ts,'Name','Experiment');
data = data(500:3500);                         % Select a relatively stable segment
data.InputName = 'Input Voltage';                   %_Input name
data.InputUnit =  'V';                          %_Input unit
data.OutputName{1} = 'Angular Velocity';              %_Output name
data.OutputUnit{1} = 'rad/s';                          %_Output unit
data.OutputName{2} = 'Current';              %_Output name
data.OutputUnit{2} = 'A';                          %_Output unit
data.Tstart = 500*Ts;                                %_Starting time
data.TimeUnit = 's';                            %_Time unit
   

%% Kalman test - test out the identified model on data
Q_Kalman = diag([1, 0.01])   ; %sum process
R_Kalman = diag([1, 10]); %sum measurement
clear estimateKalmanState % Filter algorithm
xEstimated = zeros(2, length(data.y));
yEstimated = zeros(2, length(data.y));
xIC = [0;
       ifltr(1)];
for k = 1:length(data.y)
    [xEstimated(:, k), yEstimated(:, k)] = estimateKalmanState(data.u(k), [data.y(k,1); data.y(k,2)], zmodel.A, zmodel.B, zmodel.C, Q_Kalman, R_Kalman, xIC);
end
%xEstimated=xEstimated-xIC;
t = (1:length(data.y)) * Ts + 500*Ts;
%% Plotting measured vs estimated vs filtered
figure
subplot(2,1,2)
plot(t, data.y(:,2),'color',[0.6 0.6 0.6],'linewidth',0.5)
hold on
plot(t, ifltr(500:3500),'b','linewidth',2)
hold on
plot(t, xEstimated(2,:),'r','linewidth',1.5)
hold off
grid on
legend('Measured Current','Filtered Current','Estimated Current - Kalman')
xlabel('Time (s)'); ylabel('Current [A]')
title('')
xlim([10 70])
subplot(2,1,1)
plot(t,data.y(:,1))
hold on
grid on
plot(t,xEstimated(1,:))
xlabel('Time (s)');
ylabel('Ang. velocity (rad/s)')
legend('Measured Angl. Vel.','Estimated ang vel. - Kalman')
title('')
xlim([10 70])
