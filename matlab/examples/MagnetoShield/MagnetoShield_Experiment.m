% MagnetoShield data aquisition from experiment

l=3000;                         % Experiment section length
lngth=5*l;                      % PID
port='COM12'                    % Your port
baud=250000;                     % Baud rate (2M for AVR, 250 000 for SAM3)

result=readExperiment(lngth,port,baud);
plot(result)                    % Just a preview
save result result              % Save in file