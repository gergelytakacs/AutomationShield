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
 
Np = 10; 
% Constraints
uL = 0;        % Lower input limit
uU = 100;      % Upper input limit

%penalization matrices
R=5;
Q=diag([1 0.001 1]);

[K, P] = dlqr(AI, BI, Q, R);
% Get Hessian, Gradient and F matrices of cost function
[H, G, F]=getCostFunctionMPC(AI, BI, Np, Q, R, P); 
% Set input constraints onto the system
[Ac, b0]=setConstraintsMPC(uL, uU, Np);

H = (H + H') / 2;            % Create symmetric Hessian matrix
opt = optimoptions('quadprog', 'Display', 'none');  % Turn off display

Ref=Reference; 
Ref=Ref(1:3000)';
 
N=length(Ref);
t=1:N;

X0=[0.01; 0];
X=X0;
R=Ref;
XI = 0; 

%simulation
for i = 1:N
    Xn(:,i)=[X(:,i);XI]; %update extended state vector
    u_opt = quadprog(H,G*Xn(:,i),Ac,b0,[],[],[],[],[],opt); %calculate optimal input with integrator
    U(i) = u_opt(1); %select input for each step
    X(:,i+1) = A*X(:,i) + B*U(i); %the "real" system (no integrator included)
    XI = XI + (Ref(i)-X(1,i)); % update integrator state
end

figure(1)
subplot(2,1,1)
plot(X(1,:))
hold on
plot(Ref)
axis([0,3000,0,100])
ylabel('Overpressure (hPa)')
xlabel('Sample')
legend('Output','Reference')
title('Output Y')

subplot(2,1,2)
plot(U)
axis([0,3000,0,100])
ylabel('PMW (%)')
xlabel('Sample')
title('Input U')