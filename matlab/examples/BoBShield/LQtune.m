
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

% Penalizacne matice
R=x(1);
Q=diag([x(2), x(3), x(4)]); %Q=diag([1e3, 1, 1e-6]);
%penalizacna matica, zosilnenie;
try
[K,P]= dlqr(A,B,Q,R);
catch % If it fails, use previous
  disp('error')
end
  Kp=K; %Previous valid


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

X3=[0.01; 0;0];
Xr=[Ref;zeros(2,N)];

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

      X3(:,k+1)=A*X3(:,k)+[B(1)*U3(k);sin(U3(k));0];
      Y3(:,k)=C*X3(:,k);
            
end




 