function [dx,y] = identified_nonlin_model_m(t,x,u,K,m,R,L,g,C,varargin)

y = [x(1);x(3)];
dx(1) = x(2);
dx(2) = -g+K/m*x(3)/x(1)^4-C*x(2); 
dx(3) = -R/L*x(3)+1/L*u(1)+K*1/(L*x(1)^4)*x(2);
