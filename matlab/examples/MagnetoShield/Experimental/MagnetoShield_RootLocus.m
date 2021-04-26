% This does not work! It is just there to demonstrate it!

% Please read http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlRootLocus 
% for more details.

clc; clear; close all;                                          % Close and clear all
load MagnetoShield_Models_Greybox_TF                            % Include linearized state-space model


model
zpk(model)

a=[-500 -550];
b=-600
z = a;
p = b;
k = 1;
C = zpk(z,p,k);


figure(1)
rlocus(model)

figure(2)
rlocus(C*model)

return


axis([-400 200 -200 200])
sgrid

%sgrid(zeta,wn)


[K,poles] = rlocfind(C*model)

sys_cl = feedback(model,K*C)
isstable(sys_cl)
figure(2)
step(sys_cl)



return

load MagnetoShield_Models_Greybox_TF                            % Include linearized state-space model


figure(1)
rlocus(model)
axis([-400 200 -200 200])
sgrid
%sgrid(zeta,wn)



[k,poles] = rlocfind(model)

sys_cl = feedback(k*model,1)
isstable(sys_cl)
figure(2)
step(sys_cl)