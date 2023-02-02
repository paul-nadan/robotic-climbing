addpath('terrain-generation', 'config-generation', 'robot-kinematics', ...
    'optimization', 'visualization', 'workspace-optimization', 'motor-selection');
global FRAMES ANIMATE RECORD PLOT
FRAMES; % stores animation frames for saving video file
close all;
ANIMATE = ~~1; % flag to animate robot motion
RECORD = ~~0; % flag to save animations as video files
PLOT = ~~0; % flag to plot final robot condition and path

CODE = '100101100';

config = simpleWalker(CODE, 0.2, 0.5, 0.1);
for iconfig = 1:length(robots)
    if isequal(abs(robots{iconfig,1}(1).config.joints)>0, abs(config.joints)>0)
        break
    end
end
iconfig
config = simpleWalker(CODE, 0.2, 0.5, L(iconfig));
config.mass = 3*ones(size(config.mass));
mname(allMotors{iconfig})

torque = zeros(1, 1, size(torques, 3), size(torques{iconfig,1,1},1), size(torques{iconfig,1,1},2));
for j = 1:size(torques, 3)
    torque(1,1,j,:,:) = torques{iconfig,1,j};
end
torque(abs(torque)<1e-10) = NaN;
torque = reshape(getDeviation(torque, 0)/5, [], 1);
torque2 = torque;
ignore = 0;
angles = robots{iconfig,1,1}(1).config.gait.angles;
for t = 1:length(torque)-1
    if t > ignore && (angles(t,1) ~= angles(t,2)) % shoulder joint
        if angles(t+1,1) == 45 % 2 joints
            torque2(t) = (torque(t) + torque(t+2))/2;
            torque2(t+2) = torque2(t);
            torque2(t+1) = (torque(t+1) + torque(t+3))/2;
            torque2(t+3) = torque2(t+1);
            ignore = t+3;
        else % 3 joints
            torque2(t) = (torque(t) + torque(t+3))/2;
            torque2(t+3) = torque2(t);
            torque2(t+1) = (torque(t+1) + torque(t+4))/2;
            torque2(t+4) = torque2(t+1);
            torque2(t+2) = (torque(t+2) + torque(t+5))/2;
            torque2(t+5) = torque2(t+2);
            ignore = t+5;
        end
    end
end
torque = torque2;

% config.torqueLimits = mtorque(allMotors{iconfig});
mname(optimizeMotors(torque, p.bodyMass, mname, mtorque, mmass, mcost))
config.torqueLimits = torque*totalMasses(iconfig);

sweep = [.25, .5, .75, 1, 1.25, 1.5, 1.75, 2]; % what factor of safety do we need?
% sweep = 1;
SIMULATE = 0;
SAMPLES = 10;
STEPS = 10;
SEED = 42;
scoreLabels = {'Success Rate', 'Normal', 'Tangential', '|F|^2', ...
    'Rat Mgn', 'Mag Mgn', 'Cost', 'Torque', 'Distance', 'Solve Time'};

if SIMULATE
    [meanScoresT, rawScoresT, allScoresT, seedsT, robotsT] = simulate(...
        @(var1, var2)getConfig(var1, var2, config), ...
        @getTerrain, @getScores, @averageScores, sweep, 1, SAMPLES, STEPS, ...
        0.25, 0, 0, SEED, scoreLabels, 0, []);
    torquesT = getTorques(robotsT, seedsT);
end

% plot cost vs sample legend sweep
costScores = allScoresT(:,:,:,:,7);
failScores = allScoresT(:,:,:,:,1);
costScores(isnan(costScores)) = 1.5;
failScores(costScores > 1) = 1
failCosts = costScores(~failScores);
meanCost = mean(allScoresT(:,:,:,:,7), [2,3], 'omitnan');
meanCost = reshape(meanCost, size(meanCost, 1), size(meanCost, 4));
% plot(sweep, mean(allScoresT(:,:,:,:,1), [2,3,4], 'omitnan'));
plot(sweep, mean(costScores, [2,3,4], 'omitnan'));
allScoresT(:,:,:,:,7)
xlabel('Factor of Safety')
ylabel('Success Rate')
title('Motor Torque Limit: 14-DoF')
figure
histogram(reshape(costScores(4,1,:,:), [], 1), .4:.05:1.5);
xlabel('Adhesion Cost');
ylabel('Frequency');
title('Motor Torque Limit: FoS = 1');
ylim([0,40]);
figure
histogram(reshape(costScores(end,1,:,:), [], 1), .4:.05:1.5);
xlabel('Adhesion Cost');
ylabel('Frequency');
title('Motor Torque Limit: FoS = 2');
ylim([0,40]);

%% Function definitions

% User-defined robot configuration as a function of swept parameters
function config = getConfig(var1, ~, config)
    config.torqueLimits = config.torqueLimits.*var1;
end

% User-defined terrain geometry as a function of swept parameters
function grid = getTerrain(~, ~, seed)
    grid = terrain([-1.5, 1.5], ...
        [-1.5 3.5], .01, 1*[1,1,0.5], [1, .25, 0.0625], 0, [0;-.5;0], seed);
end

% User-defined evaluation metrics as a function of robot state
function scores = getScores(robot, lastRobot, i, grid)
    [~, Fnorm, Ftang, T] = quasiStaticDynamicsKnownForce(robot, i, robot.F, grid);
    normalForce = max([Fnorm, 0]);
    tangentForce = max(Ftang);
    torque = max(vecnorm(T));
    Fnorm = max(Fnorm,0);
    Fmag = sqrt(Fnorm.^2 + Ftang.^2);
    [margin, ratioMargin, magnitudeMargin] = gripperMargin(Fnorm, Ftang);
    distance = robot.origin(2) - lastRobot.origin(2);
    scores = [~robot.fail, normalForce, tangentForce, sum(Fmag.*Fmag), max(ratioMargin), max(magnitudeMargin), margin, torque, distance];
end

function [meanScores, rawScores] = averageScores(rawScores, allScores, robots)
    meanScores = mean(rawScores, 3, 'omitnan');
    meanScores(:,:,3) = (1-meanScores(:,:,1))./meanScores(:,:,6);
    meanScores(:,:,4) = meanScores(:,:,2)./meanScores(:,:,6);
    meanScores(:,:,2) = max(rawScores(:,:,:,2), [], 3, 'omitnan');
end

function score = getDeviation(rawScore, sigma)
    meanScore = mean(rawScore, [3,4], 'omitnan');
    stdScore = std(rawScore, 0, [3,4], 'omitnan');
    score = meanScore + stdScore*sigma;
end

function torques = getTorques(robots, seeds)
    torques = cell(size(robots));
    for i = 1:size(robots, 1)
        for j = 1:size(robots, 2)
            for k = 1:size(robots, 3)
                skips = 0;
                for l = 2:length(robots{i,j,k})
                    robot = robots{i,j,k}(l);
                    if ~isfield(robot, 'skip')
                        robot.skip = 0;
                    end
                    skips = skips + robot.skip;
                    grid = getTerrain(i, j, seeds(i, j, k));
                    [~, ~, ~, T] = quasiStaticDynamicsKnownForce(robot, l+skips-1, robot.F, grid);
                    if isempty(torques{i,j,k})
                        torques{i,j,k} = zeros(length(robots{i,j,k})-1, size(T,2));
                    end
                    torques{i,j,k}(l-1,:) = abs(T(1,:));
                end
            end
        end
    end
end