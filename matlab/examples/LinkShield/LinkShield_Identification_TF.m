%   LinkShield grey-box single mode transfer function identification
% 
%   Identification of a linear second order transfer function capturing a
%   single vibration mode, on real measured data.
%
%   The example takes input-output identification results from the
%   LinkShield device, where input is requested degrees suplied to the
%   servo and output is the acceleration of the beam tip in m/s^2. Note
%   that the resulting transfer function captures acceleration behavior,
%   the position signal is out-of phase to acceleration and omega^2
%   smaller.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Tak�cs. 
%   Created on: 27.1.2020
%   Last update: 27.1.2020.  

startScript;                                    % Clears screen and variables, except allows CI testing

%% Data preprocessing
load resultID.mat                              % Read identificaiton results
Ts=0.003;                                      % Sampling
data=iddata(y,u,Ts)                            % Create identification data object
                                 
% Select an appropriate section                   
dataS  = data([1992:3971]);                    % Select data section (S)
dataSR = dataS([136:1920]);                    % Refine selection (R)
                                       
%% Transfer function estimation       

% Set up identification
Options = tfestOptions;                        % Create an options object
Options.Display = 'on';                        % Display procedure
Options.WeightingFilter = [];                  % No weighting filter
                       
% Launch and save identification
sys = tfest(dataSR, 2, 0, Options)              % Identify transfer function
save sys sys                                   % Save transfer function
 
%% Data and display

% Data
disp('---The model parameters are---')
omega= sqrt(sys.Denominator(3))               % Angular natural frequency
zeta = sys.Denominator(2)/2/omega             % Damping ratio
c=sys.Numerator(1)/(-omega^2)                 % Actuator constant

% Comparison
compare(sys,dataS)                            % Comparison of data to model
xlabel('Time')                                % X axis label
ylabel('Accel. y(t) ms^{-2}')                 % Y axis label