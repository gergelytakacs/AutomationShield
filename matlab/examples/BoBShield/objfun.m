function f = objfun(x);

[R, X, U] = LQtune(x);
f=norm(R-X((1),1:end-1))+norm(U)';