%   MAGNETOSHIELD Constants and parameters 
%
%   The code below describes the experiment how to calculate approximate  
%   number of turns of the coil of the electromagnet based on the Magnetic
%   flux density measurements and calculation of the initial guess of the
%   constant K used in the non-linear and linear model of the MagnetoShield.
%   Power supply of the electromagnet is 5 V.
%   Used Hall sensor A1302KUA-T.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   If you have found any use of this code, please cite our work in your
%   academic publications, such as theses, conference articles or journal
%   papers. A list of publications connected to the AutomationShield
%   project is available at: 
%   https://github.com/gergelytakacs/AutomationShield/wiki/Publications
%
%   Created by:       Jakub Mihalik
%   Created on:       5.12.2020
%   Last updated by:  Jakub Mihalik
%   Last update on:   22.12.2020

clear all; clc;

% CONSTANTS AND DIMENSIONS
l_coil = (12.6-2.14)/1000;  % Height of the coil [m]
r_coil = (9.8+15)/4/1000;   % Approximate radius of the coil [m]
x = 12.6/1000;    % Distance of the hall sensor from the base of the electormagnet [m]
S_coil = pi*r_coil^2;       % Surface of the electromagnet [m2]

r_mag = 0.004;              % Radius of the magnet [m]
S_mag = pi*r_mag^2;         % Surface of the magnet [m2]
V_mag = S_mag*0.002;        % Volume of the magnet [m3]

Sens = 1.3*10^(-3);     % Sensitivity of the hall sensor [V/G]
mio = 4*pi*10^(-7);     % Permeability of the vacuum
mir_f = 5000;            % Relative permeability of the iron
mi_f = mio * mir_f;     % Permeability of the environment - ferrite
mir_a = 1.00000037;     % Relative permeability of the air
mi_a = mio*mir_a;       % Permeability of the environment - air
Br = 1.26;              % Remanence of neodymium

% MEASURED DATA
U = 2.45;           % Measured Hall sensor output - no magnetic field [V]
U_field = 1.951;    % Measured Hall sensor output - magnetic field of the electomagnet[V]
Bg = (U-U_field)/Sens;    % Measured Magnetic flux density in Gauss [G]
B = Bg/10000;             % Measured Magnetic flux density in Tesla [T]
I_coil = 102.1/1000;      % Measured current through the coil [A]

% APPROXIMATE NUMBER OF TURNS OF THE ELECTROMAGNET:
N = B*2/(mi_f*I_coil)*1/(x/sqrt(r_coil^2+x^2)+(l_coil-x)/sqrt(r_coil^2+(l_coil-x)^2));

% INIIAL COMPUTATION OF THE CONSTANT K:
K = 3*S_coil*N*Br*V_mag/(2*pi);

