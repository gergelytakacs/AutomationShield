function [dx, y] = printerheat(t, x, u, alpha, beta, varargin)
% ODE file representing the dynamics of ...

% Output equations.
y = [x(1)];
% State equations.
dx = [-beta*x(1)+alpha*u(1)*u(1)+beta*300]; % Heat tranfer eq
       
  