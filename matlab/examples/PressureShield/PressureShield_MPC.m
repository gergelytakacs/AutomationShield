%   Model Predictive Control (MPC) feedback control of overpressure
%   in the PressureShield.
%
%   This example initialises and calibrates the board 
%   and starts Model Predictive Control (MPC) 
%   using a pre-defined reference. At the end 
%   the results are stored in 'responseMPC.mat' file and are shown on the 
%   figure on the screen.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
%
%   Created by Martin Staron.
%   Last update: 31.5.2021.

startScript;                % Clears screen and variables
clear estimateKalmanState;  

PressureShield = PressureShield;   % Create PressureShield object from PressureShield class
PressureShield.begin;              % Initialises shield

Ts = 0.15;                         
k = 1;                             
nextStep = 0;                       
samplingViolation = 0;              

Ref = [40 50 40 60 40 50 60 70 60 50]; % Reference overpressure in hPa

T = 750;                               % Section length
i = 0;                                 % Section counter
response = zeros(length(Ref)*T, 3);    % Preallocate output variable

X = [0; 0; 0];               % State vector
Xr = [0; 0; 0];              % Reference vector

% System state-space matrices
matA = [0.993, 0.01526; -0.003223, 0.4206];
matB = [0.008751; -0.1501];
matC = [1.74, 0.8755];

matAhat = [matA, zeros(2, 1); -matC, 1];
matBhat = [matB; 0];

% Constraints
uL = 0;        % Lower input limit
uU = 100;      % Upper input limit

% Penalisation matrices and horizon
R=8;
Q=diag([1 10 0.01]);
np = 2;                         % Prediction horizon 

Q_Kalman = [0.01, 0; 0, 0.51];  % State penalisation 
R_Kalman = 15; 
runTime = 1000;

% Get final state penalisation matrix P
[K, P] = dlqr(matAhat, matBhat, Q, R);
% Get Hessian and Gradient matrices of cost function
[H,G]=getCostFunctionMPC(matAhat,matBhat,np,Q,R,P); 
[Ac b0]=setConstraintsMPC(uL,uU,np);  

H = (H + H') / 2;                  
opt = optimoptions('quadprog', 'Display', 'none'); 
init = PressureShield.sensorRead();

tic                                 % Start measuring time
while (1)                           % Infinite loop    
    if (nextStep)                   % If step flag is enabled
        if (mod(k, T*i) == 1)       
            i = i + 1;
            if (i > length(Ref))  
                PressureShield.actuatorWrite(0.0);
                break     
            end
            Xr(1) = Ref(i);   
        end
        y = double((PressureShield.sensorRead()-init)/100); % Read overpressure
        % Predict system inputs
        u(:, 1) = quadprog(H, G*X(:, 1), Ac, b0, [], [], [], [], [], opt);
        PressureShield.actuatorWrite(u(1, 1));     % Actuate
        % Estimate states with Kalman filter
        X(1:2) = estimateKalmanState(u(1, 1), y, matA, matB, matC, Q_Kalman, R_Kalman);
        X(3) = X(3) + (Xr(1) - X(1) - Xr(1)*0.38);           
        response(k, :) = [Xr(1), y, u(1, 1)];   % Store results
        k = k + 1;                              
        nextStep = 0;                           % Disable step flag
   end                                         

     if (toc>=runTime)
             samplingViolation = 1   % Set sampling violation flag            
             PressureShield.actuatorWrite(0);
             break                   % Stop program if they do overlap
         end
        nextStep = 1;                % Enable step flag
end
toc
response = response(1:k-1, :);      % Remove unused space
save responseMPC response           % Save results in responseMPC.mat file
plotResults('responseMPC.mat')      % Plot results from responseMPC.mat file