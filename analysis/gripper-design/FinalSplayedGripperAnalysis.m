% close all
axial = true;
set(groot,'defaultLineLineWidth',4);
set(0, 'DefaultAxesLineWidth', 4);
set(groot,'defaultAxesFontSize',28);
lsize = 36; % Axis label font size
tsize = 41; % Title font size
lwidth = 2; % Legend border


x = [2, 3, 5, 10, 100];                     % Carriage numbers
theta = linspace(0, 90, 180+1)';            % Splay angles
alpha = 90 - theta;                         % Gripper displacement angle
phi = zeros(length(theta), length(x));      % Gripper loading angle results

for j = 1:length(x)
    n = x(j);                               % Number of carriages
    angles = zeros(length(theta), n);       % Individual carriage angles
    for k = 1:length(theta)
        angles(k, :) = linspace(-theta(k), theta(k), n);
    end
    fi = cosd(alpha-angles);                % Individual carriage forces
    fx = sum(fi.*cosd(angles), 2);          % Gripper tangential load
    fy = sum(fi.*sind(angles), 2);          % Gripper lateral load
    phi(:, j) = atan2d(fy, fx);             % Gripper loading angle
end

plot(theta, phi);
lsize = 36;
xlabel('Max Carriage Angle ($\alpha_{max}$)', 'FontSize', lsize);
ylabel('Max Load Angle ($\theta_{max}$)', 'FontSize', lsize);
xtickformat('%d$^\\circ$')
ytickformat('%d$^\\circ$')
legend('2 Carriages', '3 Carriages', '5 Carriages', '10 Carriages', '100 Carriages', 'location', 'northwest');
title('Splayed Gripper In-Plane Loading');
box off;
legend boxoff;