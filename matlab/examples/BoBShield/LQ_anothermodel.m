startScript;                            % Clears screen and variables, except allows CI testing              

Ts=0.01;
% load('myModel.mat')
% csystem=ss(model.A,model.B,model.C,model.D);
% %2nd order linear gray model, with no disturbance model, input in radians
% dsystem=c2d(csystem,Ts,'zoh');
% A=dsystem.A;
% B=dsystem.B;
% C=dsystem.C;
% D=dsystem.D;


%model from ..../SynologyDrive/Publications/EDUCON%202021%20BOBShield/Reading/modeling/Bolivar-Baez_2014.pdf
A=[0 1 0 0;0 0 -7 0; 0 0 0 1; -56.8 0 0 0];
B=[0 0 0 52.6]';
C=[1 0 0 0];
D=0;
csystem=ss(A,B,C,D);
dsystem=c2d(csystem,Ts,'zoh');
A=dsystem.A;
B=dsystem.B;
C=dsystem.C;
D=dsystem.D;

%integrator
 A = [A, zeros(4, 1); -C, 1];
 B = [B; 0];
 C = [C, 0];

% Penalizacne matice
R=0.1;
Q=eye(5);
%Q=diag([1e8, 0, 1]); %Q=diag([1e3, 1, 1e-6]);
%penalizacna matica, zosilnenie;
[K,P]= dlqr(A,B,Q,R);

load('PID_ver2\dataPidAll.mat')
Um=dataAll(:,3);
Ym=dataAll(:,2);
Ref=dataAll(:,1); %reference
%select "the interesting section"
 Um=Um(500:5500)';
 Ym=Ym(500:5500)';
 Ref=Ref(500:5500)'/1000;
 
 N=length(Ref);
 t=1:N;

X0=[0.01; 0;0;0;0];
X1=X0;
X2=X0;
X3=X0;
Xr=[Ref;zeros(4,N)];

 %simulacia s regulaciou 
for k=1:N
    U2(k)=-K*(X2(:,k)-Xr(:,k));
    X2(:,k+1)=A*X2(:,k)+B*U2(k);
%    X2(:,k+1)=A*X2(:,k)+[B(1)*U2(k);sin(U2(k));0];
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
 
      X3(:,k+1)=A*X3(:,k)+B*U3(k);
      %X3(:,k+1)=A*X3(:,k)+[B(1)*U3(k);sin(U3(k));0];
      Y3(:,k)=C*X3(:,k);
            
end

figure(1)
subplot(211)
plot(t,Xr(1,:),'b',t,Y2,'r',t,Y2,'g')
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