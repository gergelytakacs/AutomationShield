clc;
clear;

% Optimizer construction


%     R      Q(1)  Q(2,2)  Q(3,3)
ub = [inf      inf   inf     inf]';  % Initial guess
lb = [eps      eps   eps       eps]';  % Initial guess
x0 = [100        100     1      1]';  % Initial guess

 
options = optimoptions('patternsearch','PlotFcn',{@psplotbestf,@psplotfuncount,@psplotbestx});
options.MeshTolerance = 1E-6;
options.InitialMeshSize = 1; % Default is 1;
options.ScaleMesh=true; % Default, if problem is badly scaled
opts.AccelerateMesh = false; % Only recommended for smooth problems

fun = @objfun;
tic
[x,fval,exitflag,output] = patternsearch(fun,x0,[],[],[],[],lb,ub,[],options)
TET=toc;

save x x

x
fval
exitflag
output