% MagnetoShield data aquisition from experiment

lngth=5*1500                    % PID
port='COM24'                    % Your port

result=readExperiment(lngth,port,2000000);
plot(result)                    % Just a preview
save result result              % Save in file