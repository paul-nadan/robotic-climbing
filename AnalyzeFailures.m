x = 4;
y = 1;
n = 3;
solve = ~~1;

global PLOT ANIMATE RECORD
PLOT = 1;
ANIMATE = 1;
RECORD = 0;

seed = seeds(x, y, n);
if solve
    [meanScores1, rawScores1, allScores1, seeds1, robots1] = simulate(@getConfig, ...
        @getTerrain, @getScores, @averageScores, SWEEP1(x), SWEEP2(y), 1, STEPS, ...
        TIME_STEP, ABORT_STRIKES, IGNORE_FAILS, seed, SCORES, 0);
    robots1 = robots1{1,1,1};
else
    robots1 = robots{x,y,n};
end

grid = terrain([-1.5, 1.5], ...
    [-1.5 3.5], .01, 1*[1,1,0.5], [1, .25, 0.0625], 0, [0;-.75;0], seed);
cost = zeros(STEPS+1,1);
skips = 0;
for i = 1:STEPS+1
    robot = robots1(i);
    if isfield(robot, 'skip')
        skips = skips + robot.skip;
    end
    figure(i);
    plotTerrain(grid);
    plotRobot(robot);
    title(i);
    [F, Fnorm, Ftang, ~, N] = quasiStaticDynamicsKnownForce(robot, i+skips-1, robot.F, grid);
    Fnorm = max(Fnorm,0);
    [~, ~, c2] = gripperMargin(Fnorm, Ftang);
    cost(i) = max(c2);
    plotForces(robot, F, i+skips-1, 'g', 0.016);
    plotForces(robot, [N, [0;0;0]], i+skips-1, 'r', 0.16);
end
disp(cost);
disp([robots1.fail]');
disp([robots1.skip]');


% User-defined robot configuration as a function of swept parameters
function config = getConfig(var1, var2)
    configs = {[2,2,2,2], [3,2,2,2], [3,3,2,2], [3,3,3,2], [3,3,3,3]};
    config = quadruped(configs{3}, ...
        0.1, 0.3, {var1, [var2/2, var2/2]}, 0, 3);
%         0.1, 0.3, {var1, var2}, 0, 2);
end

% User-defined terrain geometry as a function of swept parameters
function grid = getTerrain(var1, var2, seed)
    grid = terrain([-1.5, 1.5], ...
        [-1.5 3.5], .01, 1*[1,1,0.5], [1, .25, 0.0625], 0, [0;-.75;0], seed);
end

% User-defined evaluation metrics as a function of robot state
function scores = getScores(robot, lastRobot, i, grid)
%     [F1, n1,t1,~] = quasiStaticDynamicsMargin(robot, i, grid);
%     [F2, ~,~,~] = quasiStaticDynamicsKnownForce(robot, i, robot.F, grid);
%     margin1 = gripperMargin(n1, t1);
%     deltaForce = sum(abs(F1-F2), 'all');
%     fprintf('Force difference = %f, ', deltaForce);
    [~, Fnorm, Ftang, T] = quasiStaticDynamicsKnownForce(robot, i, robot.F, grid);
    normalForce = max([Fnorm, 0]);
    tangentForce = max(Ftang);
    torque = max(vecnorm(T));
    Fnorm = max(Fnorm,0);
    Fmag = sqrt(Fnorm.^2 + Ftang.^2);
    [margin, ratioMargin, magnitudeMargin] = gripperMargin(Fnorm, Ftang);
%     fprintf('Margin improvement = %f\n', margin-margin1);
    ratio = normalForce./tangentForce;
%     pathLength = norm(robot.origin - lastRobot.origin);
    distance = robot.origin(2) - lastRobot.origin(2);
%     scores = [~robot.fail, normalForce, tangentForce, sum(Fmag.*Fmag), max(ratioMargin), max(magnitudeMargin), margin, torque, distance];
    scores = [~robot.fail, max(magnitudeMargin), NaN, NaN, torque, distance];
end

function scores = averageScores(rawScores, robots)
    scores = mean(rawScores, 3, 'omitnan');
%     scores = max(rawScores, [], 3, 'omitnan');
%     scores = min(rawScores, [], 3, 'omitnan');
%     scores = std(rawScores, 0, 3, 'omitnan');
    scores(:,:,3) = (1-scores(:,:,1))./scores(:,:,6);
    scores(:,:,4) = scores(:,:,2)./scores(:,:,6);
    scores(:,:,2) = max(rawScores(:,:,:,2), [], 3, 'omitnan');
end