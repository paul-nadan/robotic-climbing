figure();
plot(data(:,1), data(:,4), 'r');
hold on;
plot(data(:,1), data(:,7), 'g');
plot(data(:,1), data(:,10), 'b');
plot(data(:,1), data(:,13), 'm');
ylabel('Estimated Force (N)')
xlabel('Time (ms)')
legend('Foot 1', 'Foot 2', 'Foot 3', 'Foot 4')
title('Z-Component of Force')