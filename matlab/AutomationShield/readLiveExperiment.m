function result=readLiveExperiment(lngth,port,baudrate)

s = serial(port);    % Serial port handle                            
set(s,'BaudRate',baudrate); % Set speed
fopen(s);               % Open port
result=[];

sampleText = fscanf(s);   % Read once

% Temporary saving
lngth10=round(lngth/10); %10% of length
k=1;

% Save the first sample
sampleText = fscanf(s);   % Read until newline character
sampleCell = textscan(sampleText,'%f%f','Delimiter',', ');
result(1,:)=sampleCell{1}';
[r c]=size(result); % Size of the data
 
% The rest of the samples
for i=2:lngth-1;              %Skip last one
    sampleText = fscanf(s);   % Read until newline character
    sampleCell = textscan(sampleText,'%f%f','Delimiter',', ');
    if length(sampleCell{1}')==c;
        result(i,:)=sampleCell{1}';
    end
    plotLive(result(i,:))
    if i==k*lngth10; % Save at 10% increments (for long experiments)
        save resultTemp result;
        k = k + 1;
    end
end

delete 'resultTemp.mat' %delete temporary save, let user decide

fclose(s);              % Close port






