%   OptoShield system identification experiment
% 
%   Aquires data from the OptoShield then fits 1st order model to
%   identification results.
%
%   This code is part of the AutomationShield hardware and software
%   ecosystem. Visit http://www.automationshield.com for more
%   details. This code is licensed under a Creative Commons
%   Attribution-NonCommercial 4.0 International License.
% 
%   Created by Gergely Takács. 
%   Last update: 01.10.2018.

%plotOptoShield_Step
load resultID
Ts=0.005
Y=resultID(:,1);
U=resultID(:,2);

data = iddata(Y,U,Ts);      
dataselection = data([1:400]);   
Opt = procestOptions;                 
model = procest(dataselection,'P1', Opt);

compare(dataselection,model);
pause

figure(1)
[Ym, FIT, X0]=compare(data,model);
Ym=Ym.y(:,1);
plot([Y U Ym])
legend('Output','Input','Model')
xlabel('Samples ()')
ylabel('Input, Output (%)')