clear, clc
%define LQR
Ts=0.01;
load('myModel.mat')%load model
%create state-space model + continuous to discrete
csystem=ss(model.A,model.B,model.C,model.D);
dsystem=c2d(csystem,Ts,'zoh');
%assign matrices from the model
A=dsystem.A;
B=dsystem.B;
C=dsystem.C;
D=dsystem.D;
%integrator
AI = [A, zeros(2, 1); -C, 1];
BI = [B; 0];
CI = [C, 0];

%Assign extended matrices
Ad = AI;
Bd = BI;
Cd = [1 0];
Dd = [0];
dx0 = [0.01; 0]; 

% Penalty matrices
R=1E2;
Q=diag([ 1E2, 1, 1]);


umax = 30; %upper boundary
np = 40;%number of prediction steps

%add limits
Ac = [eye(np);
     -eye(np)];
b = ones(np*2,1)*umax;
%load measurede data - just to load my reference
load('dataPidAll.mat')
Ref=dataAll(:,1); %reference
Ref=Ref(500:5500)';

%simulation length
poc=length(Ref);


%% Prediction
[H,G,Ac2,Bx2,M,N] = MPC_INPUT(Ad,Bd,Q,R,np,0); %function from "zaklady prediktivnej..."

X = dx0;%initial state, I guess
XI=0; %initial integrator state
opts = optimset('Algorithm','interior-point-convex','Display','off'); %optimization options

%simulation
for i = 1:poc
    Xn(:,i)=[X(:,i);XI]; %update extended state vector
    u_opt = quadprog(H,G*Xn(:,i),Ac,b,[],[],[],[],[],opts); %calculate optimal input with integrator
    U(i) = u_opt(1); %select input for each step
    X(:,i+1) = A*X(:,i) + B*U(i); %the "real" system (no integrator included)
    XI = XI + (Ref(i)-X(1,i)); % update integrator state
end

figure(1)
subplot(2,1,1)
plot(X(1,:))
subplot(2,1,2)
plot(U)








function [H,G,Ac,Bx,M,N] = MPC_INPUT(A,B,Q,R,np,simulink)

% funkcia na vypocet matic potrebnych pre MPC
nx = length(A);
nu = size(B);
nu = nu(2);

%% alokovanie pamate
M = zeros(nx*np, nx);
N = zeros(nx*np, nu*np);
A_pom = zeros(nx);
N_pom = zeros(nx*np, nu);

%% matica predikcie dynamiky

for i = 1:np
    A_pom = A^i;
    for j = 1:nx
        M(i*nx-nx+j,:) = A_pom(j,:);
    end
end

%% matica predikcie vstupov

M_pom = [eye(nx); M(1:(np-1)*nx,:)];    % posunutie matice M dolu o nx krokov

for i = 1:np
    y1 = nx*i-(nx-1);
    y2 = y1+(nx-1);
    N_pom( y1:y2,:) = M_pom(y1:y2,:)*B;
end

for i = 1:np
    y1 = nx*i-(nx-1);
    y2 = nx*np - (i-1)*nx;
    x1 = nu*i-(nu-1);
    x2 = x1 + (nu-1);
    N(y1:np*nx , x1:x2) = N_pom(1:y2,:);
end

R_ = zeros(nu*np);
H = zeros(np*nu);
G = zeros(np*nu, nx);

K = dlqr(A,B,Q,R);
A_ = (A - B*K)';        % +/- K
Q_ = Q + K'*R*K;
Pf = dlyap(A_,Q_);

for i = 1 : nu : nu*np
    R_(i:i+(nu-1), i:i+(nu-1)) = R;
end

for i = 1 : nx : (np-1)*nx
    H = H + N(i:i+(nx-1), :)' * Q * N(i:i+(nx-1), :);
    G = G + N(i:i+(nx-1), :)' * Q * M(i:i+(nx-1), :);
end

H = H + N(np*nx-(nx-1):np*nx, :)'* Pf *N(np*nx-(nx-1):np*nx, :) + R_;
G = G + N(np*nx-(nx-1):np*nx, :)'* Pf *M(np*nx-(nx-1):np*nx, :);
Ac = N;
Bx = M;

if simulink == 1
    H = mat2vec(H);
    Ac = mat2vec(Ac);
end

end

