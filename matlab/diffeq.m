%   AutomationShield SISO difference equation
%
%   Solves a SISO difference equation at time (k), that is computes y(k)
%   for a(1)*y(n) = b(1)*x(n) + b(2)*x(n-1) + ... + b(nb+1)*x(n-nb)
%                             - a(2)*y(n-1) - ... - a(na+1)*y(n-na)
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 7.9.2020.

function y = diffeq(a,b,u)

persistent Ybuf Ubuf                           % Creates a persistent variable

if isempty(Ybuf)
    na   = length(a);                          % Determines order of a
    nb   = length(b);                          % Determines order of b         
    
    Ybuf = zeros(1,na-1);                      % Creates a buffer for y(k)   
    Ubuf = zeros(1,nb-1);                      % Creates a buffer for u(k)
end

y = sum(Ubuf.*b(2:end))-sum(Ybuf.*a(2:end));   % Computes y(k)

Ybuf = [y  Ybuf(1:end-1)];                     % Updates y(k) buffer
Ubuf = [u  Ubuf(1:end-1)];                     % Updates u(k) buffer

end
