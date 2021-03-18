clear all; close all; clc;
%define LQR
Ts=0.01;
load('myModel.mat')
csystem=ss(model.A,model.B,model.C,model.D);
%ssest, so sumom alebo bez, ale rovnako v simulacii ako v modeli. 
%fok vagy radian? 
dsystem=c2d(csystem,Ts,'zoh');
A=dsystem.A;
B=dsystem.B;
C=dsystem.C;
D=dsystem.D;
%integrator
 A = [A, zeros(2, 1); -C, 1];
 B = [B; 0];
 C = [C, 0];
 
 
load x

% Penalizacne matice
R=x(1);
Q=diag([x(2), x(3), x(4)]); %Q=diag([1e3, 1, 1e-6]);
%penalizacna matica, zosilnenie;
[K,P]= dlqr(A,B,Q,R);

K 
%load measurede data
load('PID_ver2\dataPidAll.mat')
Um=dataAll(:,3);
Ym=dataAll(:,2);
Ref=dataAll(:,1); %reference
%select "the interesting section"
 Um=Um(500:5500)';
 Ym=Ym(500:5500)';
 Ref=Ref(500:5500)';
 
 N=length(Ref);
 t=1:N;

X0=[0.01; 0;0];
X1=X0;
X2=X0;
X3=X0;
Xr=[Ref;zeros(2,N)];

%simulacia v otvorenej slucke
 for k=1:N
    X1(:,k+1)=A*X1(:,k);
    Y1(:,k)=C*X1(:,k);
end

 %simulacia s regulaciou 
for k=1:N
    U2(k)=-K*(X2(:,k)-Xr(:,k));
    %X2(:,k+1)=A*X2(:,k)+B*U2(k);
    X2(:,k+1)=A*X2(:,k)+[B(1)*U2(k);sin(U2(k));0];
    Y2(:,k)=C*X2(:,k);
end
% saturovana regulacia 
 
for k=1:N
      U3(k)=-K*(X3(:,k)-Xr(:,k));
      if U3(k)>5*pi/180
         U3(k)=5*pi/180;
      elseif U3(k)<-5*pi/180
         U3(k)=-5*pi/180;
      else
         U3(k)=U3(k);
      end;
 
      %X3(:,k+1)=A*X3(:,k)+B*U3(k);
      X3(:,k+1)=A*X3(:,k)+[B(1)*U3(k);sin(U3(k));0];
      Y3(:,k)=C*X3(:,k);
            
end

figure(1)
subplot(211)
plot(t,Xr(1,:),'b',t,Y2,'r',t,Y3,'g')
title('Priebeh výstupu Y')
legend('ref','bez satur','satur')
%axis([0 20 -0.0100 0.0100])
xlabel('Èas t (s)')
subplot(212)
plot(t,U2,'r',t,U3,'g');
title('Priebeh vstupnej velièiny U')
legend('bez satur','satur')
xlabel('Èas t (s)')
%axis([0 20 -100 100])
figure()
plot(t,U3,'g');
title('Priebeh vstupnej velièiny U')
legend('satur')
xlabel('Èas t (s)')
%%


 
% Vlastnéèísla matice
% %vykreslenie stablity bez riadenia
% figure(2)
% Z=eig(A);
% 
% Fi=(A-B*K); 
% Z2= eig(Fi);
% figure(2)
% plot(complex(Z),'r+', 'MarkerSize',20)
% hold on
% zgrid 
% axis equal;
% plot(Z2,'b+','MarkerSize',15)
% hold on; 
% zgrid; 
% axis equal
% title('Zobrazenie korenov')
%  
% % % graficke porovnanie frekvencnych charakteristik
% % figure(3)
%  
% [psdx1,f] = periodogram(Y2,[],[],0.01);
% [psdx2] = periodogram(Y1,[],[],0.01);
% [psdx3] = periodogram(Y3,[],[],0.01);
%  
% % plot them
%  
% plot(f*10000,10*log10(psdx1),'r',f*10000,10*log10(psdx2),'b',f*10000,10*log10(psdx3),'g')
% legend('regulator bez satur.','free respons','regulator satur.')
% title('Vykonova spektralna hustota')
% ylabel('Vykon/frekvencia (dB/Hz)')
% xlabel('Frekvencia (Hz)')
