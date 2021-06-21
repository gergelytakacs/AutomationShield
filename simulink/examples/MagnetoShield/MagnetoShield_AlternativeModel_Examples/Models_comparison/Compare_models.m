%   Comparison of two designed models of MagnetoShield
%
%   This script is used as comparison of two designed concepts, how to
%   think about MagnetoShield in the terms of a linear state-space model.
%   It compares "Alternative model", what is a concept based on Jakub
%   Mihalik's final thesis with the "original model" proposed in the
%   article about MagnetoShield on IEEE conference 2020 (can be found at 
%   https://github.com/gergelytakacs/AutomationShield/wiki/MagnetoShield#ident).
%   Data sample the comparison is based on has been captured during PID
%   levitation experiment. The data are independent - none of the model's
%   identifications has been executed with use of the data.

%   First are compared models in their principal form - that means without
%   estimation of the parameters, pure products of the design process.
%   Consequently, is the comparison of the identified models executed.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%
%   Created by:       Jakub Mihalik
%   Created on:       29.4.2021
%   Last updated by:  Jakub Mihalik
%   Last update on:   29.4.2021

clear; clc;
%% Load data
load Data_frequency.mat

% Identified models
load Alternative_model\MagnetoShield_Alternative_Model.mat;
model_identif_1 = model_f;
load   Original_model\MagnetoShield_Models_Greybox_SS.mat;
model_identif_2 = model;

% Non-identified models
load Alternative_model\MagnetoShield_Alternative_Model_NOT_Identified.mat;
model_NOTidentif_1 = sys;
load   Original_model\MagnetoShield_Models_Greybox_SS_NOT_Identified.mat;
model_NOTidentif_2 = sys;

% structures to compare models all at once
models = {model_identif_1 model_identif_2 model_NOTidentif_1 model_NOTidentif_2};
titles = {'Alternative model','Original model','Non-identified alternative model',...
    'Non-identified original model'};

data = dataf;
%% Compare models

for i=1:4
figure(i)
model = models{i};
compare(data,model);                            % Compare data to model
grid on;

setoptions(gcr,'FreqUnits','Hz');               % Set x-axes units
setoptions(gcr,'PhaseUnits','rad');             % Set y-axes units for phase

title(titles{i});

allLegendInFigure = findall(gcf,'type','legend');
allLegendInFigure(1).Location='southwest';
allLegendInFigure(2).Location='southwest';

end