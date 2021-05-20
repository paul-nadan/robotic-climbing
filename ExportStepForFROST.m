% close all
addpath('terrain-generation', 'config-generation', 'robot-kinematics', ...
    'optimization', 'visualization', 'discrete-model', 'dynamics');
config = spirit40(0,2);
seed = 42;
step0 = 1; % 1 is default
global ANIMATE RECORD PLOT
ANIMATE = 1;
RECORD = 1;
PLOT = 1;
STEPS = 1;

% grid = terrain([-.75, 1.25], ...
%         [-1.5 0.5], .01, [1,1,0.5], [1, .25, 0.0625], 0, [0;-.75;0], seed);

% grid = sine_terrain([.5,0;.5,0], [0.05,0;.05,0], [.23205/2,.23205/2;0,0], -1.5:.01:1.5, -1.5:.01:1.5);
% r0 = spawnRobot([0;-.5;0], eye(3), config, grid, 0);
% r0.skip = 0;
% for iStep = 1:step0-1
%     r0 = step2(r0, iStep, grid, 0);
% end
% robots = repmat(r0, STEPS + 1, 1);
% for iStep = 1:STEPS
%     robots(iStep+1) = step2(robots(iStep), step0+iStep-1, grid, 0);
% end

plotTerrain(grid);
for iStep = 2%:length(robots)
    plotRobot(robots(iStep));
end

x = zeros(22,length(robots));
joints = [7;8;9; 15;16;17; 19;20;21; 11;12;13];
for iStep = 1:length(robots)
    xi = robot2state(robots(iStep));
    x(joints, iStep) = -deg2rad(xi(10:end));
    x([7;15;19;11], iStep) = -x([7;15;19;11], iStep);
    x([2;1;3], iStep) = xi(1:3);
    x(2, iStep) = -x(2, iStep);
    x([6;4;5], iStep) = rotm2eul(robots(iStep).R0);
    x(5, iStep) = -x(5, iStep);
    robots(iStep).x = x(:,iStep);
end