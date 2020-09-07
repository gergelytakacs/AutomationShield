%   AutomationShield SISO difference equation
%
%   Solves a SISO difference equation at time (k), that is computes y(k)
%   for y(n) = b(1)*x(n) + b(2)*u(n-1) + ... + b(nb+1)*u(n-nb)
%                        - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 7.9.2020.

function y = diffeq(a,b,u,varargin)

persistent Ybuf Ubuf                           % Creates a persistent variable

if isempty(Ybuf)
    na   = length(a);                          % Determines order of a
    nb   = length(b);                          % Determines order of b
    
    if nargin == 4
        Ybuf = varargin{1}*ones(1,na-1);                      % Creates a buffer for y(k)
        Ubuf = zeros(1,nb-1);                         % Creates buffer for u(k)    else if nargin == 5
    else if nargin == 5;
            Ybuf = varargin{1}*ones(1,na-1);                      % Creates a buffer for y(k)
            Ubuf = varargin{2}*ones(1,nb-1);                      % Creates a buffer for u(k)
        else
            Ybuf = zeros(1,na-1);                      % Creates a buffer for y(k)
            Ubuf = zeros(1,nb-1);                      % Creates a buffer for u(k)
        end
    end
end

y = sum(Ubuf.*b(2:end))-sum(Ybuf.*a(2:end));   % Computes y(k)

Ybuf = [y  Ybuf(1:end-1)];                     % Updates y(k) buffer
Ubuf = [u  Ubuf(1:end-1)];                     % Updates u(k) buffer

end
