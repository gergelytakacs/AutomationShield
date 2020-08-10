function [A,B,C,D] = idef_lin_m(x01,x02,x03,K,R,L,m,Ck,Ts)
% Linear statespace representation of the system
% Terms of A and B matrices are based on the Linearization.m file

A = [0 1 0;
    -(4*K*x03)/(m*x01^5), -Ck, K/(m*x01^4);
    -(4*K*x02)/(L*x01^5), K/(L*x01^4), - R/L];
B = [0; 0; 1/L];
C = [1 0 0;0 0 1];
D = [0;0];
end