%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compare the performance of different robotic climbers on uneven terrain.
%
% USAGE INSTRUCTIONS:
%    1. Set global flags to control visualization options and indicate
%       whether to resuse previous simulation data.
%    2. Set simulation parameters to control what parameter values are
%       used, indicate the length of each trial, and provide variable
%       names for labeling plots.
%    3. Use the "sweep" variable anywhere in the code to vary the value
%       of a parameter as the simulation progresses.
%    4. Record scores by setting indices in the rawScores vector to the
%       appropriate value, i.e. "rawScores(iter, iConfig, i) = _______"
%    5. Set the indices in CONFIGURATIONS to the robot geometries being
%       compared. Use the configuration generator functions as desired.
%    6. Run the simulation and save plots as desired. To save the raw
%       simulation results save the workspace as a .mat file. Be sure
%       to clear global variables before importing a saved workspace.
%       Animations can be saved using the writeAnimation function in
%       the same folder, i.e. "writeAnimation(FRAMES{i}, 'Filename')"
%
% CONFIGURATION FORMAT:
% Robot configurations are structured as a tree of reference frames
% ("joints"), each with a position and axis of rotation defined relative to
% its parent. Each joint has an attached link, and the endpoint of the
% link ("vertex") is the origin for any child joints. Any number of
% polygonal "bodies" can also be assigned to a joint. Robot configurations
% are defined as structs with the following fields:
%       joints(:,1,j) = origin of joint j with respect to its parent
%       joints(:,2,j) = axis of rotation for joint j
%       joints(:,3,j) = link attached to joint j at zero rotation
%       count(j) = numer of child joints attached to joint j
%       parents(j) = index of the parent of joint j
%       bodies{b}(:,v) = vertex v of body b
%       iBodies(b) = index of parent joint for body b
%       limits(j,:) = lower and upper limits on the angle for joint j
%       clearance = minimum distance allowed between any body vertices and
%                   the climbing surface
%       gait = a struct specifying the robot's walking pattern (see below)
%
% GAIT FORMAT:
% Gaits specify a repeated sequence of robot states ("steps") that produce
% a desired climbing motion. Gaits are defined as structs with the
% following fields:
%       angles(j,i) = angular position of joint j during step i (degrees)
%       feet(j,i) = true if vertex j contacts the ground during step i
%       dx(:,i) = centroid displacement for step i
%       dyaw = maximum yaw correction per step (radians)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath('terrain-generation', 'config-generation', 'robot-kinematics', ...
    'optimization', 'visualization');
global FRAMES ANIMATE RECORD PLOT
FRAMES; % stores animation frames for saving video file
close all;

% USER-DEFINED VISUALIZATION PARAMETERS
ANIMATE = ~~0; % flag to animate robot motion
RECORD = ~~0; % flag to save animations as video files
PLOT = ~~0; % flag to plot final robot condition and path

% USER-DEFINED SIMULATION PARAMETERS
SEED = 42; % set the terrain seed, -1 for a random seed
% SCORES = {'Success Rate', 'Normal', 'Tangential', '|F|^2', 'Rat Mgn', 'Mag Mgn', 'Cost', 'Torque', 'Distance', 'Solve Time'}; % output variable names
SCORES = {'Success Rate', 'Cost', 'Failures Per Meter', 'Cost Per Meter', 'Torque', 'Distance', 'Solve Time'}; % output variable names
PLOT_SCORES = [1,2,3,4,6];
% PLOT_COLORS = {'r', [1 .5 0], [0 .7 0], 'b', [.5 0 .5]};
SWEEP1 = 0.15:0.02:0.25; % values for parameter being swept
SWEEP2 = 0.25:0.02:0.35;%0.1:0.05:0.3;
% SWEEP1 = 0.21;
% SWEEP2 = 0.31;
AXIS_LABELS = {'Back Leg Length (m)', 'Front Leg Length (m)'};
% AXIS_LABELS = {'Terrain Difficulty', 'Configuration'};
SAMPLES = 10; % number of duplicate samples to average at each value
STEPS = 5; % number of robot steps to simulate per trial
TIME_STEP = 0.25; % delay between frame updates for animation
ABORT_STRIKES = 0; % aborts remaining samples after this number of failures
IGNORE_FAILS = 0; % do not record any data from a failed trial
REUSE_DATA = 0; % reuse previous simulation data in the workspace
PLOT_ONLY = 0; % do not run the simulation
OPTIMIZE = ~~0; % Perform an optimization instead of a parameter sweep
ITERS = 50; % Maximum iterations for performing parameter optimization

% RUN SIMULATION
if REUSE_DATA == 1
    REUSE_DATA = robots;
end
if OPTIMIZE && ~PLOT_ONLY
    results = optimize(@getConfig, ...
        @getTerrain, @getCost, SWEEP1, SWEEP2, SAMPLES, STEPS, ...
        ABORT_STRIKES, IGNORE_FAILS, SEED, ITERS);
elseif ~OPTIMIZE && ~PLOT_ONLY
    [meanScores, rawScores, seeds, robots] = simulate(@getConfig, ...
        @getTerrain, @getScores, @averageScores, SWEEP1, SWEEP2, SAMPLES, STEPS, ...
        TIME_STEP, ABORT_STRIKES, IGNORE_FAILS, SEED, SCORES, REUSE_DATA);
end

% VISUALIZE CUMULATIVE RESULTS
if OPTIMIZE
    f = fitrgp(results.XTrace, results.ObjectiveTrace);
    preds = zeros(length(SWEEP1), length(SWEEP2));
    for a = 1:length(SWEEP1)
        for b = 1:length(SWEEP2)
            preds(a,b) = predict(f, [SWEEP1(a), SWEEP2(b)]);
        end
    end
    figure();
    imagesc(SWEEP1, SWEEP2, preds');
    xlabel(AXIS_LABELS{1});
    ylabel(AXIS_LABELS{2});
    title('Estimated Gaussian Process Model');
    colorbar;
    mse = sqrt(mean((results.ObjectiveTrace-resubPredict(f)).^2));
    fprintf('Mean squared error: %.3d\n', mse);
else
    plotResults(meanScores, SWEEP1, SWEEP2, SCORES, PLOT_SCORES, AXIS_LABELS);
end

% User-defined robot configuration as a function of swept parameters
function config = getConfig(var1, var2)
    configs = {[2,2,2,2], [3,2,2,2], [3,3,2,2], [3,3,3,2], [3,3,3,3]};
    config = quadruped(configs{3}, ...
        0.1, 0.3, {var1, [var2/2, var2/2]}, 0, 2);
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
    scores = [~robot.fail, margin, NaN, NaN, torque, distance];
end

function scores = averageScores(rawScores, robots)
    scores = mean(rawScores, 3, 'omitnan');
%     scores = max(rawScores, [], 3, 'omitnan');
%     scores = min(rawScores, [], 3, 'omitnan');
%     scores = std(rawScores, 0, 3, 'omitnan');
    scores(:,:,3) = (1-scores(:,:,1))./scores(:,:,6);
    scores(:,:,4) = scores(:,:,2)./scores(:,:,6);
end

function cost = getCost(robot, lastRobot, i, grid)
    [~, Fnorm, Ftang, T] = quasiStaticDynamicsKnownForce(robot, i, robot.F, grid);
    Fnorm = max(Fnorm,0);
    [margin, ~, ~] = gripperMargin(Fnorm, Ftang);
    distance = robot.origin(2) - lastRobot.origin(2);
    cost = min(1, max(robot.fail, margin));
%     cost = robot.fail;
end