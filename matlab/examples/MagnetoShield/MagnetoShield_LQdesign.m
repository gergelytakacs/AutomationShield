clc; clear; close all;

load MagnetoShield_Models_Greybox_SS

Ts=1E-3;                            % [s] sampling
R=[14.0,13.0,14.0,14.5,13.5,13.0]'; % [mm] Reference
T=1000;                             % [samples] Section length 

y0=13.64                            % [mm] Linearization point
u0=2.09                             % [V] Linearization point
ra=(R-y0)/1000;                     % [mm] Adjusted reference
yi=0.50; %mm

t=0:Ts:(T*length(R)-1)*Ts;
umin=0; % V
umax=12; % V
i=1; % Section counter
r=ra(1); % First reference

%% Discretization
modeld=c2d(model,Ts); % Discretize model
A=modeld.a;
B=modeld.b;
C=[1 0 0];

%% Model augmentation by integration
[ny nx]=size(C);                
[nx nu]=size(B);                 
Ai=[eye(ny,ny) -C;               
    zeros(nx,ny)  A];          
Bi=[zeros(ny,nu); B];            
Ci=[zeros(ny,nx) C];             

Qlq=diag([1E2 10 10 1]);
Rlq=1E-4;
K=dlqr(Ai,Bi,Qlq,Rlq);

x0=[0 0 0]';
X=x0;
xI=0;

for k=1:length(t)-1
    if (mod(k,(T*i)) == 0)                    % At each section change
        i = i + 1;                            % We move to the next index
        r = ra(i);                            % and use the actual, corrected reference
    end

    
    xI(k+1)=xI(k)+(r-X(1,k));                 % Integrator
    U(k)=-K*[xI(k); X(:,k)];                  % LQ 
    U(k)=constrain(U(k),umin-u0,umax-u0);     % Constrain inputs    

    X(:,k+1)=A*X(:,k)+B*U(k);                 % System model
    X(:,k+1)=X(:,k+1)+rand(1)*0.5E-4;         % Disturbance
    
    % Just data logging and model correction
    y(k)=constrain(X(1,k)*1000+y0,12.0,17.0);
    U(k)=U(k)+u0;                             % Real input (V)
    Rr(k)=R(i);                               % Original reference, just for logging
end

figure(1);

subplot(2,1,1)
plot(t(1:end-1),Rr);
hold on

plot(t(1:end-1),y);
grid on
xlabel('Time (s)')
ylabel('Position (mm)')

subplot(2,1,2)

plot(t(1:end-1),U);
grid on
xlabel('Time (s)')
ylabel('Input (V)')

