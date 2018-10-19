function plotLive(data)

persistent c i Co datap

if (isempty(c))
    i = 1;
    c =length(data);
    Co= [0         0.4470    0.7410;
        0.8500    0.3250    0.0980;
        0.9290    0.6940    0.1250;
        0.4940    0.1840    0.5560;
        0.4660    0.6740    0.1880;
        0.3010    0.7450    0.9330;
        0.6350    0.0780    0.1840];
    figure;
    grid on;
    xlabel('Samples');
    ylabel('Amplitude')
    datap=data;
else
    for j=1:c
        line([i-1 i],[datap(j) data(j)],'Color',Co(j,:));
        drawnow;
    end
    datap=data;
    i=i+1;
end

end