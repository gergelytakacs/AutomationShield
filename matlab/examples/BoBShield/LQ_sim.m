close all; clear all; clc;
Ts=0.01;
load('myModel.mat')
csystem=ss(model.A,model.B,model.C,model.D);


dsystem=c2d(csystem,Ts,'zoh');
A=dsystem.A;
B=dsystem.B;
C=dsystem.C;
D=dsystem.D;


% Penalizacne matice
R=0.00001;
Q=C'*C;
%penalizacna matica, zosilnenie;
[K,P]= dlqr(A,B,Q,R);

X0=[0.01; 0];
X1=X0;
X2=X0;
X3=X0;
t=0:Ts:5;
N=length(t);
%simulacia v otvorenej slucke
 for k=1:N
    X1(:,k+1)=A*X1(:,k);
    Y1(:,k)=C*X1(:,k);
end
 
%simulacia s regulaciou
 
for k=1:N
    U2(k)=-K*(X2(:,k));
    X2(:,k+1)=A*X2(:,k)+B*U2(k);
    Y2(:,k)=C*X2(:,k);
end
 

 
% saturovana regulacia 
 
for k=1:N
      U3(k)=-K*X3(:,k);
      if U3(k)>30*pi/180
         U3(k)=30*pi/180;
      elseif U3(k)<-30*pi/180
         U3(k)=-30*pi/180;
      else
         U3(k)=U3(k);
      end;
 
      X3(:,k+1)=A*X3(:,k)+B*U3(k);
      Y3(:,k)=C*X3(:,k);
            
end

figure(1)
subplot(211)
plot(t,Y1,'b',t,Y2,'r',t,Y3,'g')
title('Priebeh výstupu Y')
%axis([0 20 -0.0100 0.0100])
xlabel('Èas t (s)')
subplot(212)
plot(t,U2,'r',t,U3,'g');
title('Priebeh vstupnej velièiny U')
xlabel('Èas t (s)')
%axis([0 20 -100 100])



 
% Vlastnéèísla matice
%vykreslenie stablity bez riadenia
figure(2)
Z=eig(A);

Fi=(A-B*K); 
Z2= eig(Fi);
figure(2)
plot(complex(Z),'r+', 'MarkerSize',20)
hold on
zgrid 
axis equal;
plot(Z2,'b+','MarkerSize',15)
hold on; 
zgrid; 
axis equal
title('Zobrazenie korenov')
 
% graficke porovnanie frekvencnych charakteristik
figure(3)
 
[psdx1,f] = periodogram(Y2,[],[],0.01);
[psdx2] = periodogram(Y1,[],[],0.01);
[psdx3] = periodogram(Y3,[],[],0.01);
 
% plot them
 
plot(f*10000,10*log10(psdx1),'r',f*10000,10*log10(psdx2),'b',f*10000,10*log10(psdx3),'g')
legend('regulator bez satur.','free respons','regulator satur.')
title('Vykonova spektralna hustota')
ylabel('Vykon/frekvencia (dB/Hz)')
xlabel('Frekvencia (Hz)')
