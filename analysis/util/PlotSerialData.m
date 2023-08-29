data(:,1) = data(:,1) - data(1,1);

figure();
plot(data(:,1), data(:,3));
hold on;
plot(data(:,1), data(:,6));
plot(data(:,1), data(:,9));
plot(data(:,1), data(:,12));
ylabel('Tangential Force (N)')
xlabel('Time (s)')
legend('Foot 1', 'Foot 2', 'Foot 3', 'Foot 4')
% title('Y-Component of Force')
xlim([0, 26]);
ylim([-5, 20]);

% figure();
% plot(data(:,1), data(:,4), 'r');
% hold on;
% plot(data(:,1), data(:,7), 'g');
% plot(data(:,1), data(:,10), 'b');
% plot(data(:,1), data(:,13), 'm');
% ylabel('Estimated Force (N)')
% xlabel('Time (ms)')
% legend('Foot 1', 'Foot 2', 'Foot 3', 'Foot 4')
% title('Z-Component of Force')