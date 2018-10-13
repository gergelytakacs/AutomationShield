% result=readExperiment(8100,'COM22',9600);
% save resultID result

clc; clear;

load resultID

Ts=2;
Tabs=273.15;
Troom=25;
Y=resultID(:,1);
U=resultID(:,2);

data = iddata(Y,U,Ts,'Name','Printer Head');      
data.InputName = 'Voltage';
data.InputUnit =  'V';
data.OutputName = 'Temperature';
data.OutputUnit = 'oC';
data.Tstart = 0;
data.TimeUnit = 's';



model1 = nlarx(data,[1 1 0],'linear');
model2 = tf(42.2,[1200 1])
compare(data, model1, model2)



