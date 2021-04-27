result=readExperiment(6*100,'COM22',9600);
save result result

plot(result)
hold on
legend('Reference','Output','Input')
xlabel('Sample no. (-)')
ylabel('Input, output level (%)')
grid on

