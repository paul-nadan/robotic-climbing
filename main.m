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
ANIMATE = ~~1; % flag to animate robot motion
RECORD = ~~0; % flag to save animations as video files
PLOT = ~~1; % flag to plot final robot condition and path

% USER-DEFINED SIMULATION PARAMETERS
SEED = 44; % set the terrain seed, -1 for a random seed
SCORES = {'Success Rate', 'Normal', 'Tangential', '|F|^2', 'Rat Mgn', 'Mag Mgn', 'Margin', 'Torque', 'Distance', 'Solve Time'}; % output variable names
PLOT_SCORES = [1,7];
% PLOT_COLORS = {'r', [1 .5 0], [0 .7 0], 'b', [.5 0 .5]};
SWEEP1 = .15:.1:.25;%0.1:0.02:0.3; % values for parameter being swept
SWEEP2 = .1:.1:.2;
AXIS_LABELS = {'Back Leg Length (m)', 'Front Leg Length (m)'};
SAMPLES = 2; % number of duplicate samples to average at each value
STEPS = 3; % number of robot steps to simulate per trial
TIME_STEP = 0.25; % delay between frame updates for animation
ABORT_STRIKES = 0; % aborts remaining samples after this number of failures
IGNORE_FAILS = 0; % do not record any data from a failed trial

% RUN SIMULATION
[meanScores, rawScores, seeds, robots] = simulate(@getConfig, ...
    @getTerrain, @getScores, SWEEP1, SWEEP2, SAMPLES, STEPS, TIME_STEP, ...
    ABORT_STRIKES, IGNORE_FAILS, SEED, SCORES);

% EVALUATE CUMULATIVE RESULTS
plotResults(meanScores, SWEEP1, SWEEP2, SCORES, PLOT_SCORES, AXIS_LABELS);

% USER-DEFINED VISUALIZATIONS
% Plot margin vs step for 1 trial
% marginRat = stepScores(:,1,4);
% marginMag = stepScores(:,1,5);
% margin = stepScores(:,1,6);
% figure(); hold on;
% plot(marginRat, 'r');
% plot(marginMag, 'b');
% plot(margin, 'k');
% plot(1+0*margin, 'k--')
% xlabel('Step'); ylabel('Margin');
% legend('Ratio Margin', 'Magnitude Margin', 'Combined Margin', 'Maximum Threshold');
% title('Grasp Adhesion Margin');
% fprintf('Success Rate: %.1f%%\n', 100*sum(margin<=1)/STEPS);

% User-defined robot configuration as a function of swept parameters
function config = getConfig(var1, var2)
    config = quadruped([3,3,2,2], ...
        0.1, 0.3, {var1, [var2, var2]}, 0, 2);
end

% User-defined terrain geometry as a function of swept parameters
function grid = getTerrain(var1, var2, seed)
    grid = terrain([-1.5, 1.5], ...
        [-1.5 3.5], .01, [1,1,0.5], [1, .25, 0.0625], 0, [0;-.75;0], seed);
end

% User-defined evaluation metrics as a function of robot state
function scores = getScores(robot, lastRobot, i, grid)
    global COST_FUNCTION
    [F1, n1,t1,~] = quasiStaticDynamicsMargin(robot, i, grid);
    [F2, ~,~,~] = quasiStaticDynamicsKnownForce(robot, i, robot.F, grid);
    margin1 = gripperMargin(n1, t1);
    deltaForce = sum(abs(F1-F2), 'all');
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
    scores = [~robot.fail, normalForce, tangentForce, sum(Fmag.*Fmag), max(ratioMargin), max(magnitudeMargin), margin, torque, distance];
end