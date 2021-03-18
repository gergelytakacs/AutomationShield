
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
 AI = [A, zeros(2, 1); -C, 1];
 BI = [B; 0];
 CI = [C, 0];

% Penalizacne matice
R=x(1);
Q=diag([x(2), x(3), x(4)]); %Q=diag([1e3, 1, 1e-6]);
%penalizacna matica, zosilnenie;

[K,P]= dlqr(AI,BI,Q,R);
if max((abs(eig(AI-BI*K)))) >= 1
    disp('Unstable')
end



%load measurede data
load('PID_ver2\dataPidAll.mat')
Ref=dataAll(:,1); %reference
%select "the interesting section"
 Ref=Ref(500:5500)'./1000;
 
 N=length(Ref);
 t=1:N;

X3=[0; 0];
XI = 0;

% saturovana regulacia 
for k=1:N
      XI = XI + (Ref(k)-X3(1));
      U3(k)=-K*[X3(:,k); XI];
      
      % Input saturation
      if U3(k)>5*pi/180
         U3(k)=5*pi/180;
      elseif U3(k)<-5*pi/180
         U3(k)=-5*pi/180;
      end;
      

      X3(:,k+1)=A*X3(:,k)+B*U3(k);
      % Output saturation
      if X3(1,k)> 0.080
          X3(1,k)=0.080;
      elseif X3(1,k) < 0
                X3(1,k)=0.0;
      end
          
      Y3(:,k)=C*X3(:,k);
            
end

figure(10)
hold off
plot(Ref)
hold on
plot(X3(1,:))


 