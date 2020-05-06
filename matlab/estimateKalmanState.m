%    Function used to estimate internal states of linear SISO system
%    through Kalman filter equations, using system state-space matrices.
%
%    Usage:
%    [stateEstimate, outputEstimate] = estimateKalmanState(u,y,A,B,C,Q,R,xIC);
%
%    Where A,B,C are state-space matrices, Q,R are covariance matrices,
%    and u,y is current value of system input and output respectively.
%    Last argument - xIC is optional and can be used to set initial
%    values of estimated states.
%
%    To estimate a full set of data (u and y are already fully known) you can use:
%    for i = 1:length(u)
%    [stateEstimate(:,i), outputEstimate(i)] = estimateKalmanState(u(i),y(i),A,B,C,Q,R,xIC);
%    end
%
%    This code is part of the AutomationShield hardware and software
%    ecosystem. Visit http://www.automationshield.com for more
%    details. This code is licensed under a Creative Commons
%    Attribution-NonCommercial 4.0 International License.
%
%    Inspired by Phil Goddard from www.goddardconsulting.ca
%
%    Created by Peter Chmurèiak
%    Last update: 5.4.2020

function [stateEstimate, outputEstimate] = estimateKalmanState(systemInput, measuredOutput, A, B, C, Q, R, xEstimateIC)

if nargin < 7
    error('estimateKalmanState(u,y,A,B,C,Q,R) function expects at least seven input arguments!')
elseif nargin < 8                        % Optional argument - to define initial states
    xEstimateIC = zeros(length(A), 1);   % If not specified, IC will be defined as zeros
end

persistent P xEstimate             % "Static" variables to store values between function calls
if isempty(P)
    xEstimate = xEstimateIC;       % Estimated state matrix
    P = zeros(size(A));            % Error covariance matrix
end

% Prediction
xEstimate = A * xEstimate + B * systemInput; % Calculate initial "a priori" state estimate
P = A * P * A' + Q;                          % Calculate error covariance (process noise)

% Update
K = P * C' / (C * P * C' + R);               % Calculate the Kalman gain (with measurement noise)
difference = measuredOutput - C * xEstimate; % Calculate difference between measurement and estimate
xEstimate = xEstimate + K * difference;      % Calculate corrected "a posteriori" state estimate
P = (eye(size(K, 1)) - K * C) * P;           % Update error covariance matrix 

stateEstimate = xEstimate;          % Return vector of estimated states
outputEstimate = C * stateEstimate; % Return estimated measured output
end