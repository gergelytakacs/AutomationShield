% addpath C:\Users\micha\Documents\Bakalarka\Automationshield\matlab
%listArduinoLibraries
clear;
arduinoObj = arduino('COM3', 'Mega2560', 'Libraries', {'skuska/skuska'}, 'ForceBuildOn', true);

dev = addon(arduinoObj,'skuska/skuska');
%s = servo(arduinoObj, 'D9');

read (dev)
beginSensor(dev)

for angleDeg = 65:10:125
    angleRad=angleDeg/180   %Deg to Rad
    %%writePosition(s, angleRad);
    mm=readSensor(dev);
    %current_pos = readPosition(s);
    %current_pos = (current_pos/pi)*180;
    %fprintf('Current motor position is %d degrees\n', current_pos);
    fprintf('Current motor position is %d degrees\n', angleDeg);
    fprintf('Current ball position is %d mm\n', mm);
    pause(2);
end
