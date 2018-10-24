function result=readExperiment(lngth,port,baudrate)

s = serial(port);    % Serial port handle                            
set(s,'BaudRate',baudrate); % Set speed
fopen(s);               % Open port
result=[];

sampleText = fscanf(s);   % Read once
for i=1:lngth;            % Do 100 readings
sampleText = fscanf(s);   % Read until newline character
sampleCell = textscan(sampleText,'%f%f','Delimiter',', ');
result(i,:)=sampleCell{1}';
end

fclose(s);              % Close port






