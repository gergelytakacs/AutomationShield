function process_pid()
startScript;                            % Clears screen and variables, except allows CI testing              
%get data
load('dataPidAll.mat')
u=dataAll(:,3);
y=dataAll(:,2);
r=dataAll(:,1); % open loop reference obtained only by scaling the input
%%
%select "the interesting section"
 u=u(500:5500)';
 y=y(500:5500)';
 r=r(500:5500)';

%%
width=12;
height=12;
fsize=8;
fname='PID'
allAxesInFigure = findall(gcf,'type','axes');
allLegendInFigure = findall(gcf,'type','legend');
allTextInFigure = findall(gcf,'type','text')

set(allAxesInFigure, 'FontName', 'Times New Roman','FontSize',fsize)
title("")



for i=1:length(allTextInFigure)
    allTextInFigure(i).Color=[0 0 0];
end

for i=1:length(allAxesInFigure)
    allAxesInFigure(i).XColor=[0 0 0];
    allAxesInFigure(i).YColor=[0 0 0];
end






set(gcf, 'Units', 'centimeters', 'Position', [0, 2, width, height], 'PaperUnits', 'centimeters', 'PaperSize', [width, height])

%%
blue=    [0, 0.4470, 0.7410];
red =    [0.8500, 0.3250, 0.0980];
yellow = [0.9290, 0.6940, 0.1250];
grey = 0.5*[1, 1, 1];
Ts = 0.010; % [s]
Tend=24;
t = 0:Ts:Ts*(length(y)-1);
f=figure(1);
subplot(2,1,1)
plot(t,y,'Color',blue,'LineWidth',0.5,'LineStyle','-')
hold on
plot(t,r,'Color',grey,'LineWidth',0.5,'LineStyle','-')
%axis([0,Tend,12,16])
grid on
xlabel('Time (s)')
ylabel('Distance (mm)')
legend('Output y(k)', 'Reference r(k)','Location','southwest')
set(gca, 'FontName', 'Times New Roman','FontSize',fsize)

subplot(2,1,2)
plot(t,u,'Color',blue,'LineWidth',1,'LineStyle','-')
%axis([0,Tend,12,16])
grid on
xlabel('Time (s)')
ylabel('Servo gear angle (°)')
set(gca, 'FontName', 'Times New Roman','FontSize',fsize)

set(gcf, 'PaperUnits', 'centimeters');
set(gcf, 'PaperSize', [4 6]);

set(gcf, 'Units', 'centimeters', 'Position', [0, 2, width, height], 'PaperUnits', 'centimeters', 'PaperSize', [width, height])
end
%%
saveas(gcf,fname,'epsc')
saveas(gcf,fname,'jpeg')
print(gcf,fname,'-depsc');