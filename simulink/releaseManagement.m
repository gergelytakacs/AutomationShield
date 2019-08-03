clc; clear;

libNames={'AutomationShield','FloatLibrary','HeatLibrary','OptoLibrary'};
exNames={'OptoShield_InputsOutputs','OptoShield_PID_Control','OptoShield_Pulse_Identification','HeatShield_ID_Test','HeatShield_InputsOutputs','HeatShield_PID','HeatShield_PID_Simulate'};
versions=[];

disp('')
disp('=======================')
disp('')

disp('Library Model Versions:')
disp('-----------------------')
for i=1:length(libNames)
    info = Simulink.MDLInfo(libNames{i});
    versions{i}=info.ReleaseName;
    disp([libNames{i},': ',versions{i}])    
end

disp('')
disp('=======================')
disp('')

disp('Example Model Versions:')
disp('-----------------------')
for i=1:length(exNames)
    info = Simulink.MDLInfo(exNames{i});
    versions{i}=info.ReleaseName;
    disp([exNames{i},': ',versions{i}])    
end

disp('')
disp('=======================')
disp('')

if all(strcmp(versions(:),versions{1}))
    disp('All library modules and examples are saved in the same Simulink version.')
else
    disp('Warning! Not all modules and examples are saved in the same Simulink version!')
end


