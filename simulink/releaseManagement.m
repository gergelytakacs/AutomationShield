clc; clear;

libNames={'AutomationShield','Float','Heat','Opto'};
exNames={'OptoShield_InputsOutputs','OptoShield_PID_Control','OptoShield_Pulse_Identification','HeatShield_ID_Test','HeatShield_InputsOutputs','HeatShield_PID','HeatShield_PID_Simulate'};

disp('Library Model Versions:')
disp('-----------------------')
for i=1:length(libNames)
    info = Simulink.MDLInfo(libNames{i});
    disp([libNames{i},': ',info.ReleaseName])    
end

disp('=======================')

disp('Example Model Versions:')
disp('-----------------------')
for i=1:length(exNames)
    info = Simulink.MDLInfo(exNames{i});
    disp([exNames{i},': ',info.ReleaseName])    
end