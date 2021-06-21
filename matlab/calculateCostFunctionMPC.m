%    Function used to calculate the matrices H,G,F (Hessian, etc.)
%    for the quadratic cost function used in linear MPC through inputting
%    the linear state-space matrices, horizon and penalisation matrices.
% 
%    Usage:
%    [H, G, F] = calculateCostFunctionMPC(A, B, np, Q, R, P);
%
%    Where A,B are linear state-space matrices, np is prediction horizon,
%    and Q,R,P are penalisation matrices of states, input and final state
%    respectively.
%    Matrices Q and R have to be chosen manually but matrix P can then
%    be computed using matlab function:
%    [K ,P] = dlqr(A, B, Q, R);
%
%    This code is part of the AutomationShield hardware and software
%    ecosystem. Visit http://www.automationshield.com for more
%    details. This code is licensed under a Creative Commons
%    Attribution-NonCommercial 4.0 International License.
%
%    Inspired by the book "ZÁKLADY PREDIKTÍVNEHO RIADENIA"
%    (from authors: Gergely Takács, Martin Gulan, STU 2018)
%    see https://github.com/gergelytakacs/Zaklady-prediktivneho-riadenia
%    for the book in PDF (in Slovak) and the original examples.
%
%    Created by Peter Chmurciak
%    Last update: 24.4.2020

function [H, G, F] = calculateCostFunctionMPC(A, B, np, Q, R, P)
[nx, nu] = size(B);         % Get system dimensions

% Get prediction matrix M
M = zeros(np*nx, nx);       % Preallocate matrix M with zeros
for i = 1:np                % Create matrix M using powers of matrix A
    M(i*nx-nx+1:i*nx, :) = A^i; 
end

% Get prediction matrix N
N = zeros(nx*np, nu*np);    % Preallocate matrix N with zeros
NN = zeros(nx*np, nu);      % Preallocate auxilliary matrix NN with zeros
for i = 1:np                % Calculate auxilliary matrix NN
    NN(i*nx-nx+1:i*nx, :) = A^(i - 1) * B; 
end
for i = 1:np                % By shifting NN matrix create matrix N
    N(i*nx-nx+1:end, i*nu-nu+1:i*nu) = NN(1:np*nx-(i - 1)*nx, :); 
end

% Get cost function matrices H,G,F
RR = kron(eye(np), R);  % Diagonal matrix of input penalisation matrices R

H = zeros(np*nu, np*nu);    % Initialise Hessian matrix with zeros
G = zeros(np*nu, nx);       % Initialise Gradient matrix with zeros
F = Q;                      % Initialise F matrix with matrix Q

for i = 1:np - 1    % Calculate 1st to np-1 row
    H = H + N((i - 1)*nx+1:i*nx, :)' * Q * N((i - 1)*nx+1:i*nx, :);
    G = G + N((i - 1)*nx+1:i*nx, :)' * Q * M((i - 1)*nx+1:i*nx, :);
    F = F + M((i - 1)*nx+1:i*nx, :)' * Q * M((i - 1)*nx+1:i*nx, :);
end
i = np;             % Calculate last np row and output results
H = H + N((i - 1)*nx+1:i*nx, :)' * P * N((i - 1)*nx+1:i*nx, :) + RR;
G = G + N((i - 1)*nx+1:i*nx, :)' * P * M((i - 1)*nx+1:i*nx, :);
F = F + M((i - 1)*nx+1:i*nx, :)' * P * M((i - 1)*nx+1:i*nx, :);
end