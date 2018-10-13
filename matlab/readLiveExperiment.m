function result=readLiveExperiment(lngth,port,baudrate)

s = serial(port);    % Serial port handle                            
set(s,'BaudRate',baudrate); % Set speed
fopen(s);               % Open port
result=[];

sampleText = fscanf(s);   % Read once

% Save the first sample
sampleText = fscanf(s);   % Read until newline character
sampleCell = textscan(sampleText,'%f%f','Delimiter',', ');
result(1,:)=sampleCell{1}';
[r c]=size(result); % Size of the data

figure(1);
grid on;
xlabel('Samples');
ylabel('Amplitude')

Co= [0         0.4470    0.7410;
     0.8500    0.3250    0.0980;
     0.9290    0.6940    0.1250;
     0.4940    0.1840    0.5560;
     0.4660    0.6740    0.1880;
     0.3010    0.7450    0.9330;
     0.6350    0.0780    0.1840];
 
% The rest of the samples
for i=2:lngth;            % Do 100 readings
    sampleText = fscanf(s);   % Read until newline character
    sampleCell = textscan(sampleText,'%f%f','Delimiter',', ');
    result(i,:)=sampleCell{1}';
    for j=1:c
        line([i-2 i-1],[result(i-1,j) result(i,j)],'Color',Co(j,:))
        drawnow
    end
end

fclose(s);              % Close port






