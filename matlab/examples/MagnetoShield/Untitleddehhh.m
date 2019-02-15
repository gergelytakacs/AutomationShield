clc; clear all; clf;

Ts=5;                                 % Vzorkovanie [s]
run=150;                              % Dlzka simulacie
rr=[80 50];                           % Ziadana teplota
Q=eye(2); R=1; np=15;                 % Vahy a horizont
uh=10; ul=0;                          % Min a max vstup

k=10; tau=60;                         % Parametre modelu
Ac=-1/tau; Bc=k/tau; Cc=1; Dc=0;      % Spojity model
[A,B,C,D]=c2dm(Ac,Bc,Cc,Dc,Ts);       % Diskretny model
vymennik=ss(Ac,Bc,Cc,Dc);             % Objekt modelu

At=[1 -C;                             % Rozsirene A    
    0  A];                            % Dynamika + intg.
Bt=[0; B];                            % Rozsirene B
X=25'; xI=0;                          % Pociatoc. stav.                     

% Offline MPC
[K,P]=iterdlqr(At,Bt,Q,R,100);        % Konc. vahovanie  
[H,G]=ucelovafunkcia(At,Bt,np,Q,R,P); % Ucelova funkcia
[Ac bc]=obmedzenia(ul,uh,np);         % Obmedzenia u

% Vypnut varovania
H=(H+H')/2;                                     % Sym. H
o=optimoptions('quadprog','Display','none');    % Vypnut
warning('off','Control:analysis:LsimStartTime') % Lsim

% Simulacia (a online MPC)                              
for i=1:run                              % Simul. slucka
 if i<=run/2; r=rr(1); else r=rr(2); end % Referencia   
 xI=xI+(r-X(i));                         % Integrator
 U(:,i)=quadprog(H,G*[xI; X(:,i)],Ac,bc,[],[],[],[],[],o);
 [y,t,x]=lsim(vymennik,[U(1,i) U(1,i)],[(i-1) i]*Ts,X(:,i));
 X(:,i+1)=x(end,:)';                     % Aktualny stav
end
