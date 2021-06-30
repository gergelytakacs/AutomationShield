%    Function used to apply input and/or state constraints on the linear
%    state-space system matrices used in linear MPC.
%
%    Usage:
%    [Ac, b0, B0] = applyConstraintsMPC(uL, uU, xL, xU, np, A, B);
%
%    Where uL,uU are values of lower and upper boundaries on input and 
%    xL,xU are are values of lower and upper boundaries on states, 
%    np is prediction horizon and A,B are linear state-space matrices.
%
%    Depending on the number of inputs to the function also only input
%    constraints can be applied:
%    [Ac, b0] = applyConstraintsMPC(uL, uU, np);
% 
%    Or only state constraints can be applied:
%    [Ac, b0, B0] = applyConstraintsMPC(xL, xU, np, A, B);
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

function [Ac, b0, B0] = applyConstraintsMPC(p1, p2, p3, p4, p5, p6, p7)
switch nargin
    case 3  % Only input constraints
            % [Ac, b0] = setConstraintsMPC(uL, uU, np);
        nu = length(p2);                % Number of inputs
        Ac = [eye(p3*nu); -eye(p3*nu)]; % Create matrix Ac
        One = [];                       % Empty matrix One
        for i = 1:p3                    % Throughout horizon
            One = [One; eye(nu)];       % Fill matrix One
        end                             
        b0 = [One * p2; -One * p1];     % Create matrix b0
        B0 = [];                        % B0 is empty matrix
        
    case 5  % Only state constraints
            % [Ac, b0, B0] = setConstraintsMPC(xL, xU, np, A, B);
        [nx, nu] = size(p5);    % Get system dimensions
        % Get prediction matrix M
        M = zeros(p3*nx, nx);   % Fill matrix M with zeros        
        for i = 1:p3            % Create matrix M using powers of matrix A          
            M(i*nx-nx+1:i*nx, :) = p4^i;
        end        
        % Get prediction matrix N
        N = zeros(nx*p3, nu*p3);    % Fill matrix N with zeros
        NN = zeros(nx*p3, nu);      % Fill auxilliary matrix NN with zeros
        for i = 1:p3                % Calculate auxilliary matrix NN
            NN(i*nx-nx+1:i*nx, :) = p4^(i - 1) * p5;
        end
        for i = 1:p3                % By shifting NN matrix create matrix N
            N(i*nx-nx+1:end, i*nu-nu+1:i*nu) = NN(1:p3*nx-(i - 1)*nx, :);
        end
        Ac = [N; -N];                   % Create matrix Ac
        B0 = [-M; M];                   % Create matrix B0
        One = [];                       % Empty matrix One
        for i = 1:p3                    % Throughout horizon
            One = [One; eye(nx)];       % Fill matrix One
        end 
        b0 = [One * p2; -One * p1];     % Create matrix b0

    case 7  % Combination of constraints
            % [Ac, b0, B0] = setConstraintsMPC(uL, uU, xL, xU, np, A, B);
        [nx, nu] = size(p7);                % Get system dimensions
        % Input constraints 
        Acu = [eye(p5*nu); -eye(p5*nu)];    % Create matrix Acu
        One = [];                           % Empty matrix One
        for i = 1:p5                        % Throughout horizon
            One = [One; eye(nu)];           % Fill matrix One
        end
        b0u = [One * p2; -One * p1];        % Create matrix b0u
        % State constraints 
        % Get prediction matrix M
        M = zeros(p5*nx, nx);       % Fill matrix M with zeros
        for i = 1:p5                % Create matrix M using powers of matrix A
            M(i*nx-nx+1:i*nx, :) = p6^i;
        end
        N = zeros(nx*p5, nu*p5);    % Fill matrix N with zeros
        NN = zeros(nx*p5, nu);      % Fill auxilliary matrix NN with zeros        
        for i = 1:p5                % Calculate auxilliary matrix NN
            NN(i*nx-nx+1:i*nx, :) = p6^(i - 1) * p7;
        end        
        for i = 1:p5                % By shifting NN matrix create matrix N
            N(i*nx-nx+1:end, i*nu-nu+1:i*nu) = NN(1:p5*nx-(i - 1)*nx, :);
        end
        Acx = [N; -N];                  % Create matrix Acx
        B0x = [-M; M];                  % Create matrix B0x
        One = [];                       % Empty matrix One
        for i = 1:p5                    % Throughout horizon
            One = [One; eye(nx)];       % Fill matrix One
        end 
        b0x = [One * p4; -One * p3];    % Create matrix b0x
        % Combine constraints
        Ac = [Acu; Acx];                    % Create final matrix Ac
        b0 = [b0u; b0x];                    % Create final matrix b0
        B0 = [zeros(2*p5*nu, nx); B0x];     % Create final matrix B0
    otherwise
        disp('Invalid number of inputs!');
end
end