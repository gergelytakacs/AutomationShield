clc; clear;                                    % Clear screen and memory of transfer functions

load sys;
Ts=0.005;

omega=sqrt(sys.denominator(3));
zeta=sys.denominator(2)/2/omega;

omega_c=sqrt(sys.denominator(3));
zeta_c=0.04;
k=20000;
k=1
num=k;
den=[1, 2*zeta_c*omega_c omega_c^2];
C=tf(num,den);
cl=feedback(sys,-C);
isstable(cl)

step(100*sys,100*cl,15)
legend('Open','Closed');

CD=c2d(C,Ts);
M = idpoly(CD,'NoiseVariance',0) % Difference equation
M.B
M.F