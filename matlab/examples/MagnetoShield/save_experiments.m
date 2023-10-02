lq = importfile('LQ_due.txt');
mpc = importfile('MPC_due.txt');
empc = importfile('EMPC_due.txt');

LQ.r = lq(:,1); 
LQ.y = lq(:,2); 
LQ.u = lq(:,3); 

MPC.r = mpc(:,1); 
MPC.y = mpc(:,2); 
MPC.u = mpc(:,3); 

EMPC.r = empc(:,1); 
EMPC.y = empc(:,2); 
EMPC.u = empc(:,3); 

save MagnetoShield_Experiments_due LQ EMPC MPC
%%
subplot(2,1,1)
plot(LQ.y)
hold on
plot(MPC.y)
plot(EMPC.y)
plot(LQ.r(1:end-1))
hold off
legend('LQ','MPC','EMPC')

subplot(2,1,2)
plot(LQ.u)
hold on
plot(MPC.u)
plot(EMPC.u)
hold off
