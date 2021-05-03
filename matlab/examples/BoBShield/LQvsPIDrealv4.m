startScript;                            % Clears screen and variables, except allows CI testing              

%define LQR
Ts=0.01;
load('mymodel.mat')
csystem=ss(model.A,model.B,model.C,model.D);
%ssest, so sumom alebo bez, ale rovnako v simulacii ako v modeli. 
%fok vagy radian? 
dsystem=c2d(csystem,Ts,'zoh');
A=dsystem.A;
B=dsystem.B;
C=dsystem.C;
D=dsystem.D;
%integrator
AI = [A, zeros(2, 1); -C, 1];
BI = [B; 0];
CI = [C, 0];

% Penalizacne matice
R=1E2;
Q=diag([ 1E2, 1, 1]);
%penalizacna matica, zosilnenie;
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



 %simulacia s regulaciou 
for k=1:N
    
    U(k)=-K*[X(:,k); XI]; % Rozsireny stav + "observer"    
    
          if U(k)>5*pi/180
         U(k)=5*pi/180;
      elseif U(k)<-5*pi/180
         U(k)=-5*pi/180;;
      end;

    X(:,k+1)=A*X(:,k)+B*U(k); % "Realny system"
    Y(:,k)=C*X(:,k); % "Realny system"
    XI = XI + (R(k)-Y(k)); % Integracia chyby
end

figure(1)
subplot(211)
%plot(t,Xr(1,:),'b',t,Y2,'r',t,Y3,'g')
plot(t,R,'b',t,Y,'r')
title('Priebeh výstupu Y')

xlabel('Cas t (s)')
subplot(212)
plot(t,U,'r');
title('Priebeh vstupnej veliciny U')
xlabel('Cas t (s)')