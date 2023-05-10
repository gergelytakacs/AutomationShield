clear; close all; clc

%% Load identified state space model
load LinkShield_SSID.mat

R = [0.0, pi/2, -pi/4];
T = 1000;
i = 1;

t=0:Ts:(T*length(R)-1)*Ts;

r=[R(1);0;0;0];


integrator = 0; 

%% Discretize model
A = model.A;
B = model.B;
C = model.C;
D = model.D;

SYSC = ss(A,B,C,D);
SYSD = c2d(SYSC,Ts);

AD = SYSD.A;
BD = SYSD.B;
CD = SYSD.C;
DD = SYSD.D;

%% Kalman noise covariance matrices
R_Kalman = [5.808465952461661e-03 0;
            0 1.266009518986769e-07];

Q_Kalman = diag([250 100 1 100]);

%% Create integrator states matrices

[ny, ~ ]=size(C); 
[nx, nu]=size(B);

Ai=[eye(ny,ny) -CD;                                              % Augmenting A by an integrator
    zeros(nx,ny)  AD];
Bi=[zeros(ny,nu); BD];                                           % Augmenting B by the integrator
Ci=[zeros(ny,nx) CD];                                            % Augmenting C by the integrator

%% LQ gain calculation

if integrator
Rlq= 1e-2;
%Qlq=diag([1 1.2 0 0 0 0]);
Qlq=diag([1e-2 1e-10 1e2 1e3 1e-2 1e-2]);

[K, P] = dlqr(Ai, Bi, Qlq, Rlq);
else 
Rlq = 1;
%Qlq = diag([1e0 3e2 1e-15 1e-15]);
Qlq = diag([125 1 1 50]);


[K, P] = dlqr(AD, BD, Qlq, Rlq);
end

%% Simulation

x0 = [0 0 0 0]';
X = x0;
xI = [0 0]';
y = [0 0]';

x1 = 0;
x2 = 0;
x1p = 0;
x2p = 0;

umin = -5;
umax =  5;

Y = zeros(2,2);
U = zeros(1,length(t));

if integrator
for k = 1:length(t)-1

    if (mod(k,(T*i)) == 0)                                      % At each section change
        i = i + 1;                                              % We move to the next index
        r = [R(i); 0];                                              % and use the actual, corrected reference
    end

    % state estimate by difference
    x1 = y(1,k)-r(1);
    x2 = y(2,k)-r(2);
    x3 = (x1 - x1p)/Ts;
    x4 = (x2 - x2p)/Ts;
    x1p = x1;
    x2p = x2;

    xhat = [x1; x2; x3; x4];

    xI(:,k+1) = xI(:,k)+(r-[x1; x2]);                                       % Integrator state
    U(k+1) = -K*[xI(:,k); xhat];                                 % LQ compensation for the integrator augmented linear part
    U(k+1) = constrain(U(k+1),umin,umax);                         % Constrain inputs like on the real system

    % Model
    X(:,k+1)=AD*X(:,k)+BD*U(k+1);                            % System model
    y(:,k+1)=CD * X(:,k+1) ;  % Position measurement on the real system
   

end

else
for k = 1:length(t)-1

    if (mod(k,(T*i)) == 0)                                      % At each section change
        i = i + 1;                                              % We move to the next index
        r = [R(i); 0; 0; 0];                                              % and use the actual, corrected reference
    end

    x1 = y(1,k);
    x2 = y(2,k);
    x3 = (x1 - x1p)/Ts;
    x4 = (x2 - x2p)/Ts;
    x1p = x1;
    x2p = x2;
    
    xhat = [x1; x2; x3; x4];

    u = r - xhat;

    U(k) = K*u;
    U(k) = constrain(U(k),umin,umax);
    x(:,k+1) = AD*xhat+BD*U(k);
    y(:,k+1) = CD*x(:,k+1);
 rplot(k)= r(1,1);
end
end

%% Plotting results

figure('Name','Simulacia')


subplot(3,1,1)
plot(y(2,:),'DisplayName','y2')
ylabel("Uhol (Rad)")
title("Alpha")

subplot(3,1,2)
plot(y(1,:),'DisplayName','y1')
hold on
title("Theta, R")
plot(rplot)
ylabel("Uhol (Rad)")
subplot(3,1,3)
plot(U)
title("U")
ylabel("Napätie (V)")

%% Print matrices in C/C++ format

printSSMatrix(K,'K')

printSSMatrix(AD,'A')
printSSMatrix(BD,'B')
printSSMatrix(CD,'C')
printSSMatrix(Q_Kalman,'Q_Kalman')
printSSMatrix(R_Kalman,'R_Kalman')
