maxScores = max(allScores(:,:,:,2:5,:), [], 4);
meanScores = mean(maxScores, 3);

torques = zeros(10,6);
for i = 1:6
    torques(:,i) = meanScores(i,i,1,1,1:10);
end
imagesc(1:10, SWEEP2, torques'); colorbar; set(gca,'YDir','normal');
xlabel('Joint');
ylabel('Leg Length');
title('Motor Torque - Horizontal Gravity');