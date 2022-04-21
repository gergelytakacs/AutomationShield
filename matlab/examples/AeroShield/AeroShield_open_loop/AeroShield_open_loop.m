% clear all objects and variables
clear all;
clc 

% initialisation of AeroShield library 
AeroShield=AeroShield;

% create an arduino object + hall sensor object 
AeroShield.begin();

% calibration
startAngle= AeroShield.calibration(); 
lastAngle=startAngle+2048; % save startangle value + 2048(180°).(for mapping purpose)

% variables for counting time for plotting purpose
time = 0;
count = 0;

angle = 0;          % variable for angle value
potentiometer = 0;  % variable for potentiometer value 

% plot variables 
yyaxis right                            % set plotting to right axis 
plotGraph = plot(time,angle,'-r' )      % plotting angle value
ylabel('Angle (°)','FontSize',15);      % label settings
xlabel('Time (s)','FontSize',15);       % label settings
hold on                                 % hold on makes sure all of the variables are plotted

yyaxis left                             % Set plotting to left axis
plotGraph1 = plot(time,potentiometer,'-b') % plotting potentiometer value
title('Pendulum plot','FontSize',15);      % title settings   
ylabel('Percent','FontSize',15)            % label settings
legend('Potentiometer value','Pendulum angle')   % legend for plots
grid('on');                      % grid for plot 'off' to turn off grid

tic                              % time keeping

while ishandle(plotGraph)           % loop will run until plot is closed
pwm = AeroShield.referenceRead();   % read potentiometer value
AeroShield.actuatorWrite(pwm);      % actuate 

RAW= AeroShield.getRawAngle();      % read raw angle value
angle_ = map(RAW, startAngle, lastAngle, 0, 180); % map raw value to degree °
count = count + 1;                              % cycle counter
time(count) = toc;                              % time keeping
angle(count) = angle_(1);                       % angle value in time
percenta= mapped(pwm, 0.0, 5.0, 0.0, 100.0);    % map pwm to percent 
potentiometer(count) = percenta(1);             % pententiometer value in time
set(plotGraph,'XData',time,'YData',angle);      % plot first data 
set(plotGraph1,'XData',time,'YData',potentiometer); % plot second data 
axis([time(count)-5 time(count) 0 100]);        % "running" x axis settingsň

    if (angle_ > 110)                            % if angle of pendulum bigger than 110°
        AeroShield.actuatorWrite(0.0);      % stop the motor 
        disp('Angle of pendulum too high. AeroShield is turned off')
        break                               % stop the program
    end
end             

clear a;
