% MagnetoShield data aquisition from experiment

%lngth=5*1500;                   % AVR (Uno)
lngth=5*3000;                   % SAM (Due, Zero)
port='COM12';                   % Your port

%baud=2000000;                   % AVR (Uno)
baud=250000;                    % AVR (Due)
result=readExperiment(lngth,port,baud);
plot(result)                    % Just a preview
save result result              % Save in file