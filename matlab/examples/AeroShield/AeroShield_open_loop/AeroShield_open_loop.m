% clear all objects and variables
clear all;
clear a;
clc

% create an arduino object
a = arduino('com3', 'Mega2560');

% create an sensor object
as5600 = device(a,'I2CAddress',0x36);

% configurate pins 
configurePin(a,'D5', 'DigitalOutput')

% calibration
write(as5600, 0x0c, 'uint8');  % send command for reading angle
write(as5600, 0x0d, 'uint8');  % send command for reading angle
data = read(as5600, 1, 'uint16'); % read answer from sensor
startangle=data;     % save startangle
lastangle=data+2048; % save startangle + 180° (for mapping purpose)

% variables for counting time 
time = 0;
count = 0;

uhol = 0; % variable for angle
potentiometer = 0; % variable for potentiometer value 

% plot variables 
plotTitle = 'Pendulum plot';  % plot title
xLabel = 'Time (s)';                 % x-axis label1 
yLabel = 'Angle (°)';                % y-axis label
yLabelRight = 'Percent';             % x-axis label2 
legend1 = 'potentiometer value'      % legend lable1
legend2 = 'pendulum angle'           % legend lable2
yMax  = 180                          % y Maximum Value
yMin  = 10                           % y minimum Value
plotGrid = 'on';                     % 'off' to turn off grid
min = 10;                            % set y-min
max = 100;                           % set y-max
yyaxis right                         % Set plotting to right axis 
plotGraph = plot(time,uhol,'-r' )    % plotting angle variable
ylabel(yLabel,'FontSize',15);        % font settings
xlabel(xLabel,'FontSize',15);        % font settings
hold on                              %hold on makes sure all of the channels are plotted

yyaxis left                          % Set plotting to left axis
plotGraph1 = plot(time,potentiometer,'-b') % plotting potentiometer variable
title(plotTitle,'FontSize',15);      % font settings   
ylabel(yLabelRight,'FontSize',15)    % font settings 
legend(legend1,legend2)              % legend for plots
%axis([yMin yMax min max]);          % axis definition
grid(plotGrid);                      % grid for plot

tic                                  % time keeping

while ishandle(plotGraph)      % loop when Plot is Active, will run until plot is closed
pwm = readVoltage(a, 'A3');          % read potentiometer value
writePWMVoltage(a, 'D5', pwm);       % actuate 

write(as5600, 0x0c, 'uint8');        % send command for reading angle
write(as5600, 0x0d, 'uint8');        % send command for reading angle
data = read(as5600, 1, 'uint16');    % read answer from sensor

uholl = mapped(data, startangle, lastangle, 0, 180); % extern mapping function 
 count = count + 1;                         % cycle counter
 time(count) = toc;                         % time keeping
uhol(count) = uholl(1);                     % angle value in time
percenta= mapped(pwm, 0.0, 5.0, 0.0, 100.0);% extern mapping function 
potentiometer(count) = percenta(1);         % pententiometer value in time
set(plotGraph,'XData',time,'YData',uhol);   % plot first data 
set(plotGraph1,'XData',time,'YData',potentiometer); % plot second data 
axis([time(count)-5 time(count) min max]);  % "running" x axis settings
%pause(0.01);
end

delete(a);
disp('Plot Closed and arduino object has been deleted');


