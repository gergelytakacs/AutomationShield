%   FloatShield linear state-space identification with LQ gain calculation.
%
%   Identification of a first-principles based grey-box model using
%   experimental data followed by LQ gain calculation and Kalman filter
%   testing. Optional output are matrices for Arduino LQ example.  
% 
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Martin Gulan, Gergely Takács and Peter Chmurèiak.
%   Last update: 22.4.2020.

clc;
clear all;
close all;

%% Data preprocessing
load resultID.mat                               % Read identification results

Ts = 0.025;                                     % Sampling period
data = iddata(y, u, Ts, 'Name', 'Experiment');  % Create identification data object
data = data(4530:5151);                         % Select a relatively stable segment

data = detrend(data);               % Remove offset and linear trends
data.InputName = 'Fan Power';       % Input name
data.InputUnit = '%';               % Input unit
data.OutputName = 'Ball Position';  % Output name
data.OutputUnit = 'mm';             % Output unit
data.Tstart = 0;                    % Starting time
data.TimeUnit = 's';                % Time unit

%% Initial guess of model parameters
m = 0.00384;        % [kg] Weight of the ball
r = 0.015;          % [m] Radius of the ball
cd = 0.47;          % [-] Drag coefficient
ro = 1.23;          % [kg/m3] Density of air
% A = pi*r^2;       % [m2] Exposed area of the ball; circle's...
A = 2 * pi * r^2;   % ...or half of a sphere's area
g = 9.81;           % [m/s2] Gravitataional acceleration
vEq = sqrt(m*g/(0.5 * cd * ro * A)); % [m/s] Theoretical equilibrium air velocity

k = 7.78;               % Fan's gain
tau = vEq / (2 * g);    % Fan's time constant
tau2 = 0.6;             % Ball's time constant

h0 = data.y(1, 1);                          % Initial ball position estimate
dh0 = (data.y(2, 1) - data.y(1, 1)) / Ts;   % Initial ball velocity estimate
v0 = 0;                                     % Initial air velocity estimate

%% State-space model identification settings
% Unknonwn parameters
alpha = 1 / tau;
beta = 1 / tau2;
gamma = k / tau;

% State-transition matrix
A = [0,      1,      0; ...
     0, -alpha,  alpha; ...
     0,      0,  -beta];

% Input matrix
B =     [0; ...
         0; ...
     gamma];

% Output matrix
C = [1, 0, 0];

% No direct feed-through
D = 0;

% Disturbance matrix
K = zeros(3, 1);

% Initial conditions
x0 = [h0; dh0; v0];

% Construct state-space representation
sys = idss(A, B, C, D, K, x0, 0);

% Mark the free parameters
sys.Structure.A.Free = [0, 0, 0; ...
                        0, 0, 0; ...
                        0, 0, 1];

sys.Structure.B.Free = [0; ...
                        0; ...
                        1];

sys.Structure.C.Free = false;
sys.Structure.D.Free = false;

% Estimate disturbance model and initial states
sys.DisturbanceModel = 'estimate';
sys.InitialState = 'estimate';

% Set identification options
Options = ssestOptions;             % State-space estimation options
Options.Display = 'on';             % Show progress
Options.Focus = 'simulation';       % Focus on simulation
Options.EnforceStability = true;    % Enforce model stability
Options.InitialState = 'estimate';  % Estimate initial condition
Options.SearchMethod = 'lsqnonlin'; % 'auto'/'gn'/'gna'/'lm'/'grad'/'lsqnonlin'/'fmincon'

% Identify model
estimatedModel = ssest(data, sys, Options) % Run estimation procedure

% Save the identified continuous state-space model
% save FloatShield_GreyboxModel_LinearSS estimatedModel

%% Calculate LQ gain with included integrator
% Create continuous state-space object
continuousSSModel = ss(estimatedModel.A, estimatedModel.B, estimatedModel.C, estimatedModel.D);

% Option to discretize the model with different Ts than default
discretizationTs = Ts;

% Discretize continuous model and get discrete state-space model
discreteSSModel = c2d(continuousSSModel, discretizationTs, 'zoh');

% Extract individual matrices
matA = discreteSSModel.A;
matB = discreteSSModel.B;
matC = discreteSSModel.C;
matD = discreteSSModel.D;

% Modify discrete state-space matrices to include integrator state
matAhat = [matA, zeros(3, 1); -matC, 1];
matBhat = [matB; 0];
matChat = [matC, 0];

% Set Q,R penalisation matrices
matQhat = diag([1, 1, 1000000, 100]);
matRhat = 10000000;

% Calculate LQ gain that includes integrator state
matKhat = dlqr(matAhat, matBhat, matQhat, matRhat);

%% Kalman test - test out the identified model on data
% Set Q,R error covariance matrices
Q_Kalman = diag([5, 1000, 1000]);
R_Kalman = 25;

% Save individual matrices for FloatShield_LQ example
% save(fprintf('%s%.0f%s','FloatShield_LinearSS_Discrete_Matrices_',...
%     discretizationTs*1000,'ms'),'matA','matB','matC','matD','matKhat',...
%     'Q_Kalman','R_Kalman')

% Filter and estimate states from identification data
clear estimateKalmanState
xEstimated = zeros(3, length(data.y));
yEstimated = zeros(1, length(data.y));
xIC = [data.y(1); (data.y(2) - data.y(1)) / Ts; 0];
for i = 1:length(data.y)
    [xEstimated(:, i), yEstimated(:, i)] = estimateKalmanState(data.u(i), data.y(i), matA, matB, matC, Q_Kalman, R_Kalman, xIC);
end

% Filter and estimate states from raw original measurement
clear estimateKalmanState
xEstimatedWhole = zeros(3, length(y));
yEstimatedWhole = zeros(1, length(y));
xICWhole = [y(1); (y(2) - y(1)) / Ts; 0];
for i = 1:length(y)
    [xEstimatedWhole(:, i), yEstimatedWhole(:, i)] = estimateKalmanState(u(i), y(i), matA, matB, matC, Q_Kalman, R_Kalman, xICWhole);
end

% Calculate velocity by differentiating y-values from identification data
velFromData = [diff(data.y)', 0] ./ Ts;

% Calculate velocity by differentiating y-values from filtered identification data
velFromEstimate = [diff(yEstimated), 0] ./ Ts;

%% Plot test results
% Plot figures as tabs in separate window
set(0,'DefaultFigureWindowStyle','docked');

% Plot the fit of model with data
figure
compare(data, estimatedModel);      
grid on                            

% Test of filtering of measured data
figure
subplot(2, 1, 1)
plot(data.y, 'y', 'LineWidth', 1.5)
hold on
plot(yEstimated, 'k', 'LineWidth', 1)
hold off
xlim([-25, length(yEstimated) + 25])
ylim([min(yEstimated) - 25, max(yEstimated) + 25])
legend('Original', 'Estimate')
xlabel('k'); ylabel('mm')
title('Kalman filter test on identification data')
subplot(2, 1, 2)
plot(y, 'y', 'LineWidth', 1.5)
hold on
plot(yEstimatedWhole, 'k', 'LineWidth', 1)
hold off
xlim([-25, length(yEstimatedWhole) + 25])
ylim([min(yEstimatedWhole) - 25, max(yEstimatedWhole) + 25])
xlabel('k'); ylabel('mm')
title('Kalman filter test on original measurement')

% Test on differentiated vs estimated ball velocities
figure
plot(velFromData, 'b', 'LineWidth', 1.5)
hold on
plot(velFromEstimate, 'y', 'LineWidth', 1.5)
plot(xEstimated(2, :), 'k', 'LineWidth', 1.5)
xlim([-25, length(velFromData) + 25])
ylim([min(velFromData) - 5, max(velFromData) + 5])
xlabel('k'); ylabel('mm/s')
title('Ball velocity differentiated and estimated')
legend('Velocity differentiated from raw data', ...
    'Velocity differentiated from Kalman filtered data', ...
    'Velocity estimated by Kalman filter', 'Location', 'SouthEast')
grid on

% Estimated ball velocity in detailed view
figure
plot(xEstimated(2, :), 'b', 'LineWidth', 1.5)
xlabel('k'); ylabel('mm/s')
xlim([-25, length(xEstimated(2, :)) + 25])
ylim([min(xEstimated(2, :)) - 5, max(xEstimated(2, :)) + 5])
title('Ball velocity estimated by Kalman filter')
grid on

% Estimated air velocity in detailed view
figure
plot(xEstimated(3, :), 'b', 'LineWidth', 1.5)
xlim([-25, length(xEstimated(3, :)) + 25])
ylim([min(xEstimated(3, :)) - 5, max(xEstimated(3, :)) + 5])
xlabel('k'); ylabel('mm/s')
title('Air velocity estimated by Kalman filter')
grid on

% Plot figures normally
set(0,'DefaultFigureWindowStyle','normal');

return % Comment this line out if the model is satisfactory for Arduino

%% Print matrices in specific format to be used in Arduino example
disp(' ')
disp('Discrete SS Matrices: ')
printSSMatrix(matA, 'A')
printSSMatrix(matB, 'B')
printSSMatrix(matC, 'C')
printSSMatrix(Q_Kalman, 'Q_Kalman')
printSSMatrix(R_Kalman, 'R_Kalman')
printSSMatrix(matKhat, 'K')

% Printing function defined
function printSSMatrix(SSmatrix, name)
matrixSize = size(SSmatrix);
vector = SSmatrix';
vector = vector(:)';
vector = round(vector, 5);
fprintf('BLA::Matrix<%d, %d> %s%s', matrixSize(1), matrixSize(2), name, ' = {');
for i = 1:length(vector)
    fprintf('%.5g', vector(i));
    if i < length(vector)
        fprintf('%s', ', ');
    end
end
fprintf('%s', '};');
fprintf('\n');
end