startScript;                            % Clears screen and variables, except allows CI testing              

%define LQR
Ts=0.01;
load('myModel.mat')
csystem=ss(model.A,model.B,model.C,model.D);
dsystem=c2d(csystem,Ts,'zoh');
A=dsystem.A;
B=dsystem.B;
C=dsystem.C;
D=dsystem.D;
%integrator

AI = [A, zeros(2, 1); -C, 1];
BI = [B; 0];
CI = [-C, 0];

% Penalty matrices
R=1E2;
Q=diag([ 1E2, 1, 1]);

[K,P]= dlqr(AI,BI,Q,R);
 
%load measurede data
load('PID_ver2\dataPidAll.mat')
Um=dataAll(:,3);
Ym=dataAll(:,2);
Ref=dataAll(:,1); %reference
%select "the interesting section"
 Ref=Ref(500:5500)';
 
 N=length(Ref);
 t=1:N;

X0=[0.01; 0];
X=X0;
R=Ref./1000;
XI = 0; % Integrator state

 yshift = 0.0546; % Detrend
 ushift = -0.0047; % Detrend



 %simulation
for k=1:N
    
    U(k)=-K*[X(:,k); XI]; % extended state + "observer"    
    
          if U(k)>5*pi/180
         U(k)=5*pi/180;
      elseif U(k)<-5*pi/180
         U(k)=-5*pi/180;;
      end;

    X(:,k+1)=A*X(:,k)+B*U(k); % "Real system"
    Y(:,k)=C*X(:,k); % "Real system"
    XI = XI + (R(k)-Y(k));
end

blue=    [0, 0.4470, 0.7410];
red =    [0.8500, 0.3250, 0.0980];
yellow = [0.9290, 0.6940, 0.1250];
grey = 0.5*[1, 1, 1];
figure(1)
subplot(211)
%plot(t,Xr(1,:),'b',t,Y2,'r',t,Y3,'g')
t = 0:Ts:Ts*(length(Y)-1);
plot(t,R,'b',t,Y,'b')
grid on
xlabel('Time (s)')
ylabel('Distance (mm)')
xlim([0 50])
subplot(212)
plot(t,rad2deg(U),'Color',blue,'LineWidth',0.5,'LineStyle','-');
grid on
xlabel('Time (s)')
ylabel('Servo angle (°)')
xlim([0 50])

%figure(2)

Ts = 0.010; % [s]
Tend=24;
t = 0:Ts:Ts*(length(Y)-1);
f=figure(1);
subplot(2,1,1)
plot(t,Y*1000,'Color',blue,'LineWidth',0.5,'LineStyle','-')
hold on
plot(t,R*1000,'Color',grey,'LineWidth',0.5,'LineStyle','-')
%axis([0,Tend,12,16])
grid on
xlabel('Time (s)')
ylabel('Distance (mm)')
legend('Output y(k)', 'Reference r(k)','Location','southwest')
set(gca, 'FontName', 'Times New Roman','FontSize',8)