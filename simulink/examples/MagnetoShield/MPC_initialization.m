clear;clc;
load MagnetoShield_Models_Greybox_SS.mat
Ts = 0.001;                 % sampling
sysd = c2d(model,Ts);
A = sysd.a;
B = sysd.b;
C = [1 0 0];
D = 0;

%%

h = 5;                     % horizont

[m,n] = size(A);            % dimensions of A matrix
nb = size(B,2);             % width of B matrix
M = zeros(m*h,n);           % M = [A A^2 A^3 ...]'
N = zeros(m*h,nb*h);        % N = [B 0 0; AB B 0; A^2B AB B]
for j=1:h
    % Matrix M creation
    M((1:m)+m*(j-1),:) = A^j;
    % Matrix N creation
    k=1;
    for i = j:h
        N((1:m)+m*(i-1),j) = (A^(k-1)*B);
        k=k+1;
    end
end

