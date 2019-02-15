clc; clear; close all;

load MagnetoShield_Models_Greybox_SS

Ts=1E-3;                            % [s] sampling
R=[14.0,13.0,14.0,14.5,13.5,13.0]'; % [mm] Reference
T=1000;                             % [samples] Section length 

y0=13.64                            % [mm] Linearization point
i0=0.0032                           % [A] Linearization point
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
y=[17 0]';

for k=1:length(t)-1
    % Experiment (simulation) profile
    Rr(k)=R(i);                               % Original reference, just for logging
    if (mod(k,(T*i)) == 0)                    % At each section change
        i = i + 1;                            % We move to the next index
        r = ra(i);                            % and use the actual, corrected reference
    end
    
    % Measurement, reconstructing system states      
    x1=y(1,k)/1000-y0/1000;
    x2=X(2,k);
    x2=y(2,k)/1000-i0;
    
    % Control
    xI(k+1)=xI(k)+(r-x1);                      % Integrator
    U(k)=-K*[xI(k); X(:,k)]+u0;                % LQ compensated for linear part
    U(k)=constrain(U(k),umin,umax);            % Constrain inputs    


    % Model
    X(:,k+1)=A*X(:,k)+B*(U(k)-u0);             % System model
    X(1,k+1)=constrain(X(1,k+1),12E-3-y0*1E-3,17E-3-y0*1E-3); % Cage limits
    X(3,k+1)=constrain(X(3,k+1),-i0,60E-3-i0); % No reverse current        
    X(:,k+1)=X(:,k+1)+rand(1)*0.5E-4;          % Add random position disturbance
    y(1,k+1)=X(1,k)*1000+y0;                     % Position measurement
    y(2,k+1)=(X(3,k)+i0)*1000;                   % Current measurement 
  
end

figure(1);

subplot(2,1,1)
plot(t(1:end-1),Rr);
hold on

plot(t(1:end-1),y(1,1:end-1));
grid on
xlabel('Time (s)')
ylabel('Position (mm)')

subplot(2,1,2)

plot(t(1:end-1),U);
grid on
xlabel('Time (s)')
ylabel('Input (V)')

