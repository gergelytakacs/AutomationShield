startScript;

load('Reference.mat')  
load('BlackBoxModel.mat')

dsystem=disSystem2;
A=dsystem.A;
B=dsystem.B;
C=dsystem.C;
D=dsystem.D;

%integrator
 AI = [A, zeros(2, 1); -C, 1];
 BI = [B; 0];
 CI = [C, 0];
 
%penalization matrices
R=100;
Q=diag([1 10 0.01]);

[K, P] = dlqr(AI,BI,Q,R)    %LQ gain calculation

Ref=Reference; 
Ref=Ref(1:3000)';
 
N=length(Ref);
t=1:N;

X0=[0.01; 0];
X=X0;
R=Ref;
XI = 0; 



%simulation
for k=1:N
    
    U(k)=-K*[X(:,k); XI];
    
      if U(k)>100
        U(k)=100;
      elseif U(k)<0
        U(k)=0;;
      end;

    X(:,k+1)=A*X(:,k)+B*U(k);
    Y(:,k)=C*X(:,k);
    XI = XI + (R(k)-Y(k)); 
end

figure(1)
subplot(211)

plot(t,R,'b',t,Y,'r')
title('Output Y')

xlabel('Time (s)')
subplot(212)
plot(t,U,'r');
title('Input U')
xlabel('Time (s)')