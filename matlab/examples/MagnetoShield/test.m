Ts = 0.004
Kp = 3.0;
Ti = 0.1;
Td = 0.02;

P=Kp;
I=(Kp*Ts/Ti);
D=(Kp*Td)/Ts;

C=tf([(P+I+D) (-P+I-2*D) D],[1 -1 0],Ts)

Ku=1;
Kh=2;
Pc=tf(-Ku,[1 Kh]);
P=c2d(Pc,Ts)