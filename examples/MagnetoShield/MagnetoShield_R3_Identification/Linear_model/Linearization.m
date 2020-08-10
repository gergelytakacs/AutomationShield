%% Linearization
syms x1 x2 x3 x01 x02 x03 R L K u g m C

% System state variables [position; velocity; current]
x = [x1; x2; x3];

% Non-linear statespace system structure
dx1 = x2;
dx2 = -g+K/m*x3/x1^4-C*x2;
dx3 = K*1/(L*x1^4)*x2-R/L*x3+1/L*u;

% Jacobian matrix of first derivatives
J = jacobian([dx1 dx2 dx3],[x1 x2 x3]);

% Matrix A of the linear system
A = subs(J,[x1 x2 x3],[x01 x02 x03]);
% Matrix B of the linear system
B = [diff(dx1,u);diff(dx2,u);diff(dx3,u)];

 