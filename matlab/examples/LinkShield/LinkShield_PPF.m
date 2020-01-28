%   LinkShield NAF/PPF controller computation
% 
%   This file computes a negagtive acceleration feedback (positive position
%   feedback controller for the LinkShield device.
%
%   The file reads the single mode vibration model, from which it needs the
%   assumed angular natural frequency of the system. From this constructs
%   a PPF controller by compensating for the fact that the device reads 
%   acceleration signal instead of position. The transfer function is 
%   discretized and turned into a difference equation form. The open
%   and closed-loop response of the system is compared in simulation.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Created on: 27.1.2020
%   Last update: 27.1.2020.  


clc; clear;                                    % Clear screen and memory

%% Set up problem 

Ts=0.005;                                      % Choose transfer function
load sys;                                      % Load identified system
omega=sqrt(sys.denominator(3));                % Load natural frequency

omega_c=omega;                                 % Controller freq. = system freq.
zeta_c=0.04;                                   % Controller damping
g=2;                                           % Choose unit for realization
%g=1;                                          % Choose scaled for simulation

%% Create NAF/PPF controller

num=g*omega_c^2;                               % TF numerator
den=[1, 2*zeta_c*omega_c omega_c^2];           % TF denominator
C=tf(num,den);                                 % Define transfer function
bode(C);                                       % Bode plot of C
CD=c2d(C,Ts);                                  % Discretization of TF

M = idpoly(CD,'NoiseVariance',0)               % Difference equation
disp('---Difference equation for realization')
M.B
M.F

%% Simulate controller

cl=feedback(sys,-C);                          % Create feedback loop
disp('---Is it stable?---')
isstable(cl)                                  % Stability test

step(90*sys,90*cl,15)                         % 90 deg step for 15 sec.
legend('Open','Closed');                      % Labels

