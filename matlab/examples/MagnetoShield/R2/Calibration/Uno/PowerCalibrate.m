% MagnetoShield data acquisition from experiment


if isfile('PowerCal.mat')                       % Does this file exist
     load PowerCal                              % Load saved measurement
     disp("Previous measurement exists.")       % Print
else
     disp("Creating new measurement. Wait...")  % Print
     lngth=4095;                                % Experiment length (10 bit DAC)
     port='COM10';                              % Your port
     baud=4800;                                 % Baud rate 
     PowerCal=readExperiment(lngth,port,baud);  % Reading the experiment
     save PowerCal PowerCal;                    % Save it in a file   
     disp("Done.")                              % Print "Done."
end 
   
DAC=PowerCal(:,1);                             % Extract DAC levels
U=PowerCal(:,2);                               % Extract Voltage
I=PowerCal(:,3);                               % Extract Current

%% Plot DAC vs. Voltage and DAC vs. Current

figure(1);

subplot(1,2,1)
plot(DAC,U)
xlabel('DAC Level')
ylabel('Coil Voltage (V)')

subplot(1,2,2)
plot(DAC,I)
xlabel('DAC Level')
ylabel('Coil Current (I)')

%% Fitting curves
sat=max(find(U < mean(U(end-400:end))/1.0001));  % Saturation point
DACsat=DAC(1:sat);
Usat=U(1:sat);
Isat=I(1:sat);

% Voltage to DAC
[X,Y] = prepareCurveData( Usat, DACsat ); % Center and scale
[fitU2DAC, gofU2DAC] = fit( X, Y, 'poly3');
disp("Fit for Voltage to DAC levels:")
fitU2DAC
vectorToC([fitU2DAC.p1, fitU2DAC.p2, fitU2DAC.p3, fitU2DAC.p4], 'U2DAC', 'float')

% Current to DAC
[X, Y] = prepareCurveData( Isat(500:end), DACsat(500:end) ); % Center and scale
[fitI2DAC, gofI2DAC] = fit( X, Y, 'poly3');
disp("Fit for Current to DAC levels:")
fitI2DAC
vectorToC([fitI2DAC.p1, fitI2DAC.p2, fitI2DAC.p3, fitI2DAC.p4], 'I2DAC', 'float')

%% Showing curve fitting results


figure(2);

subplot(1,2,1)
plot(U,DAC)
hold on
plot(U,feval(fitU2DAC,U))
ylabel('DAC Level')
xlabel('Coil Voltage (V)')
grid on
legend(["Measured","Fit"])

subplot(1,2,2)
plot(I,DAC)
hold on
plot(I,feval(fitI2DAC,I))
ylabel('DAC Level')
xlabel('Coil Current (I)')
legend(["Measured","Fit"])
grid on
