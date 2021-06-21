%   FloatShield Kalman filter estimation of identification data.
%
%   Can be used for modifying the Q,R noise covariance matrices
%   and observing what effect it will have on the filtered data.
%   Kalman filtration is based on the linear state-space matrices
%   acquired from the gray box identification example.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Peter Chmurciak.
%   Last update: 23.5.2020.

startScript;                                    % Clears screen and variables, except allows CI testing

%% Data preprocessing
load resultID.mat                               % Read identification results
Ts = 0.025;                                     % Sampling period
data = iddata(y, u, Ts, 'Name', 'Experiment');  % Create identification data object
data = data(4530:5151);                         % Select a relatively stable segment
data = detrend(data);                           % Remove offset and linear trends
% Load discrete state-space matrices
load FloatShield_LinearSS_Discrete_Matrices_25ms.mat    

%% Kalman test - test out the identified model on data
% Set Q,R noise covariance matrices
Q_Kalman = diag([5, 1000, 1000]);
R_Kalman = 25;

% Filter and estimate states from identification data
clear estimateKalmanState
xEstimated = zeros(3, length(data.y));
yEstimated = zeros(1, length(data.y));
xIC = [data.y(1); (data.y(2) - data.y(1)) / Ts; 0];
for i = 1:length(data.y)
    [xEstimated(:, i), yEstimated(:, i)] = estimateKalmanState(data.u(i), data.y(i), matA, matB, matC, Q_Kalman, R_Kalman, xIC);
end

% Calculate velocity by differentiating y-values from identification data
velFromData = [diff(data.y)', 0] ./ Ts;

% Calculate velocity by differentiating y-values from filtered identification data
velFromEstimate = [diff(yEstimated), 0] ./ Ts;

%% Plot test results
% Create time vector
t = (1:length(data.y)) * Ts;

% Test of filtering of measured data
figure
plot(t, data.y, 'y', 'LineWidth', 1.5)
hold on
plot(t, yEstimated, 'k', 'LineWidth', 1)
hold off
grid on
xlim([t(1), t(end)])
ylim([min(yEstimated) - 25, max(yEstimated) + 25])
legend('Original', 'Estimate')
xlabel('Time (s)'); ylabel('Ball Position (mm)')
title('Kalman filter test on identification data')

% Test on differentiated vs estimated ball velocities
figure
plot(t,velFromData, 'b', 'LineWidth', 1.5)
hold on
plot(t,velFromEstimate, 'y', 'LineWidth', 1.5)
plot(t,xEstimated(2, :), 'k', 'LineWidth', 1.5)
xlim([t(1), t(end)])
ylim([min(velFromData) - 5, max(velFromData) + 5])
xlabel('Time (s)'); ylabel('Ball Velocity (mm/s)')
title('Ball velocity differentiated and estimated')
legend('Velocity differentiated from raw data', ...
    'Velocity differentiated from Kalman filtered data', ...
    'Velocity estimated by Kalman filter', 'Location', 'SouthEast')
grid on

% Estimated ball velocity in detailed view
figure
plot(t,xEstimated(2, :), 'b', 'LineWidth', 1.5)
xlabel('Time (s)'); ylabel('Ball Velocity (mm/s)')
xlim([t(1), t(end)])
ylim([min(xEstimated(2, :)) - 5, max(xEstimated(2, :)) + 5])
title('Ball velocity estimated by Kalman filter')
grid on

% Estimated air velocity in detailed view
figure
plot(t,xEstimated(3, :), 'b', 'LineWidth', 1.5)
xlim([t(1), t(end)])
ylim([min(xEstimated(3, :)) - 5, max(xEstimated(3, :)) + 5])
xlabel('Time (s)'); ylabel('Air Velocity (mm/s)')
title('Air velocity estimated by Kalman filter')
grid on

return % Comment this line out if the estimation is satisfactory for Arduino

%% Print matrices in specific format to be used in Arduino example
printBLAMatrix(Q_Kalman, 'Q_Kalman', R_Kalman, 'R_Kalman', 'BLA_Kalman_Matrices')