clear all

lqi = importfile('LQ_mega.txt');
empc = importfile('EMPC_mega_N5.txt');

subplot(2, 1, 1)

plot(rad2deg(lqi(:,1)))
hold on
plot(rad2deg(lqi(:,2)))
plot(rad2deg(empc(:,2)))
legend('Ref','LQI','EMPC (N=5)')
hold off

subplot(2, 1, 2)
plot(lqi(:,3))
hold on
plot(empc(:,3))
legend('LQI','EMPC (N=5)')
hold off