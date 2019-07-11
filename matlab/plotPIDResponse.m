%   AutomationShield function for plotting system PID response.
%
%   Works with data saved in .mat file in format of matrix with three columns - [r y u].
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Peter Chmurciak.
%   Last update: 10.7.2019.

function plotPIDResponse(aFile, aFig)        % Function definition
data = load(aFile);                          % Loads data to variable
r = data.response(:, 1);                     % Separates each column
y = data.response(:, 2);
u = data.response(:, 3);
k = 1:length(r);                             % Vector of sample indexes

maxValue = max([max(r), max(y), max(u)]);    % Finds extreme values
minValue = min([min(r), min(y), min(u)]);
range = maxValue - minValue;                 % Calculates data range

figure(aFig)                                                     % Plots on specified figure                                                 
plot(k, u, 'Color', [0.4660, 0.6740, 0.1880], 'LineWidth', 1)    % Plots input
hold on
plot(k, y, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 1)    % Plots output
stairs(k, r, 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1)       % Plots reference
hold off
xlim([k(1), k(end)])                                             % Limits range of x-axis
ylim([minValue - range / 25, maxValue + range / 25])             % Limits range of y-axis
title('Plot of system PID response')                             % Figure description
legend('Input u(k)', 'Output y(k)', 'Reference r(k)', 'Location', 'northwest')
xlabel('k')
ylabel('u(k)   y(k)   r(k)')
end