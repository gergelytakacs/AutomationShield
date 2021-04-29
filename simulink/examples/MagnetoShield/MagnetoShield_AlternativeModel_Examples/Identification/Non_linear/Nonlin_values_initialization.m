%   Initialiyation for non-linear alternative model simulation
%
%   This script is used as initialization script for non-linear model
%   simulations and experiments. The 'Ts' defines sampling period.
%   The script also provides values of parameters suitable for non-linear
%   model. Values are based on the identification results of the linear
%   model and consequently adjusted, based on the experiments with 
%   "MagnetoShield_PID_Nonlinear_Observer", to make model similar to
%   real system.
%   Non-linear model is based on Jakub Mihalik's thesis work which can be
%   found here:
%   https://github.com/gergelytakacs/AutomationShield/wiki/Publications
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%
%   Created by:       Jakub Mihalik
%   Created on:       15.4.2021
%   Last updated by:  Jakub Mihalik
%   Last update on:   15.4.2021

clear; clc;

Ts = 0.0015;
g = 9.81;
m = 0.74e-3;
L = 0.39071;
R = 213.5237;
Km = 1.8752e-08;
Ke = 4.4449e-07;
C = 0;