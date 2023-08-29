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
    'optimization', 'visualization', 'design-optimization');
global FRAMES ANIMATE RECORD PLOT
FRAMES; % stores animation frames for saving video file
close all;

% USER-DEFINED VISUALIZATION PARAMETERS
ANIMATE = ~~1; % flag to animate robot motion
RECORD = ~~1; % flag to save animations as video files
PLOT = ~~1; % flag to plot final robot condition and path

% USER-DEFINED SIMULATION PARAMETERS
SEED = 48; % set the terrain seed, -1 for a random seed
SCORES = {'Success Rate', 'Normal', 'Tangential', '|F|^2', 'Rat Mgn', 'Mag Mgn', 'Cost', 'Torque', 'Distance (m)', 'Solve Time'}; % output variable names
% SCORES = {'Success Rate', 'Cost', 'Obstacle Traversal', 'Steps', 'Torque', 'Distance', 'Solve Time'}; % output variable names
% SCORES = {'Success Rate', 'Cost', 'Fails/Distance', 'Cost/Distance', 'Torque', 'Distance', 'Solve Time'}; % output variable names
% SCORES = {'T1','T2','T3','T4','T5','T6','T7','T8','T9','T10','Time'};
PLOT_SCORES = [1, 7, 9];
% PLOT_COLORS = {'r', [1 .5 0], [0 .7 0], 'b', [.5 0 .5]};
% SWEEP1 = 0.1:0.05:0.35; % values for parameter being swept
% SWEEP2 = 0.1:0.05:0.35;
% SWEEP1 = 0:511;
% SWEEP1 = SWEEP1(dof<=14);
% SWEEP1 = bin2dec('100101100');
SWEEP1 = 15;
SWEEP2 = 45;
% SWEEP1 = 0.15:.02:.25; % values for parameter being swept
% SWEEP2 = 0.25:.02:.35;
% AXIS_LABELS = {'Back Leg Length (m)', 'Front Leg Length (m)'};
AXIS_LABELS = {'Maximum Force Angle (deg)', 'Offset Angle (deg)'};
% AXIS_LABELS = {'Terrain Difficulty', 'Configuration'};
SAMPLES = 1; % number of duplicate samples to average at each value
STEPS = 20; % number of robot steps to simulate per trial
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
    [meanScores, rawScores, allScores, seeds, robots] = simulate(@getConfig, ...
        @getTerrain, @getScores, @averageScores, SWEEP1, SWEEP2, SAMPLES, STEPS, ...
        TIME_STEP, ABORT_STRIKES, IGNORE_FAILS, SEED, SCORES, REUSE_DATA, []);
end

rad2deg([robots{1}.G])

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
%     return
    plotResults(meanScores, SWEEP1, SWEEP2, SCORES, PLOT_SCORES, AXIS_LABELS);
    return
    hold on
    clear rows
    scorei = 2;
    [~,I] = sort(dof(dof<=14));
    stdev = std(allScores(:,:,:,:,scorei), 0, [3,4], 'omitnan');
    for index = 1:size(meanScores,1)
        i = I(index);
        dof_i = sum(robots{i,1}(1).config.count);
        code = dec2bin(SWEEP1(i)) == '1';
        code = [zeros(1,9-length(code)),code];
        codestring = num2str(code);
        codestring = codestring(codestring~= ' ');
        segs = 1+(sum(code(1:3))>0)*(1+code(8));
        e = stdev(i)/sqrt(STEPS*SAMPLES);
        notail = code;
        notail(7) = 0;
        notail = num2str(notail);
        notail = notail(:,notail(1,:)~= ' ');
        i2 = find(all(codes == notail,2));
        color = 'r';
        if sum(code(1:3))
            color = 'b';
        end
        if code(8)
            color = 'g';
        end
        colors = ['r','b','m','k','r','b','m','k','r','b','m'];
        color = colors(dof_i-7);
%         color = colors(1+code(7));
%         if sum(code(1:3)) ~= 1
%             continue
%         end
%         color = colors(find(code(1:3)));
%         if ~code(7)
%             continue
%         end
%         color = colors(code(9)+1);
        pry = 'PRY';
        if meanScores(i,1) ~= 1
            plot(index, meanScores(i,scorei), [color,'x'], 'markersize', 20, 'linewidth',3);
        end
%         if any(index == [2,6,18,25,63])
%         if any(index == [3,12,31,41,62,91,74])
        if code(7)
            plot(index, meanScores(i,scorei), [color,'o'], 'markersize', 10, 'linewidth',1);
        end
        s = errorbar(index, meanScores(i,scorei), e, [color,'.'], 'markersize', 20);
        s.DataTipTemplate.DataTipRows(1).Label = 'i';
        s.DataTipTemplate.DataTipRows(2).Label = SCORES{scorei};
        rows(1) = dataTipTextRow('DoF',dof_i);
        rows(2) = dataTipTextRow('Legs',4+2*code(9));
        rows(3) = dataTipTextRow('Segments',segs);
        rows(4) = dataTipTextRow('Body Joints',{(pry(code(1:3)>0))});
        rows(5) = dataTipTextRow('Knees',{code(4:6)});
        rows(6) = dataTipTextRow('Tail',code(7));
        rows(7) = dataTipTextRow('Index',SWEEP1(i));
        rows(8) = dataTipTextRow('Code',{codestring});
        s.DataTipTemplate.DataTipRows(end+1:end+length(rows)) = rows;
    end
    title('Cost');
%     plot([1, 120], [0,0],'k--');
    xlabel('Index');
    ylabel(SCORES{scorei});
end

% User-defined robot configuration as a function of swept parameters
function config = getConfig(var1, var2)
%     configs = {[2,2,2,2], [3,2,2,2], [3,3,2,2], [3,3,3,2], [3,3,3,3]};
%     config = quadruped(configs{5}, ...
%         0.1, 0.3, [var1/2, var2/2], 0, 2);
%     config = quadruped(configs{3}, ...
%         0.1, 0.3, {var1, [var2/2, var2/2]}, 0, 2);
%     config = hexapod(0.1, 0.3, var1, 1);
%         0.1, 0.3, {var1, var2}, 0, 2);
%     code = dec2bin(var1) == '1';
    code = '100101100' == '1';
    code = [zeros(1,9-length(code)),code];
    config = simpleWalker(code, .2, .4, .18, 4, .4);
    config.swivel2 = deg2rad(var1);
    config.swivelOffset = deg2rad(var2)*config.swivelOffset;
    config.torqueLimits = [4.1;1.8;1.8; 4.1;1.8;1.8; 4.1; 4.1;1.8;1.8; 4.1;1.8;1.8; 1.8]/1.5;
end

% User-defined terrain geometry as a function of swept parameters
function grid = getTerrain(var1, var2, seed)
    grid = terrain([-1.5, 1.5], ...
        [-1.5 3.5], .01, [1,1,0.5], [1, .25, 0.0625], 0, [0;-.5;0], seed);
%     SAMPLES = 7;
%     h = (var1-(SAMPLES+1)/2)*0.05;
%     h = (var1-1)*0.05;
%     h = 0.15;
%     grid = obstacle([-1, 1], ...
%         [-1 1], .01, h, 0.1, [0;-.75;0], seed);
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
    footTravel = min(robot.vertices(2,any(robot.gait.feet,2)));
    [margin, ratioMargin, magnitudeMargin] = gripperMargin(Fnorm, Ftang);
%     fprintf('Margin improvement = %f\n', margin-margin1);
    ratio = normalForce./tangentForce;
%     pathLength = norm(robot.origin - lastRobot.origin);
    distance = robot.origin(2) - lastRobot.origin(2);
    scores = [~robot.fail, normalForce, tangentForce, sum(Fmag.*Fmag), max(ratioMargin), max(magnitudeMargin), margin, torque, distance];
%     scores = [~robot.fail, max(magnitudeMargin), footTravel, NaN, torque, distance];
%     scores = abs(T(1,:));
end

function [meanScores, rawScores] = averageScores(rawScores, allScores, robots)
    meanScores = mean(rawScores, 3, 'omitnan');
%     scores = max(rawScores, [], 3, 'omitnan');
%     scores = min(rawScores, [], 3, 'omitnan');
%     scores = std(rawScores, 0, 3, 'omitnan');
    meanScores(:,:,3) = (1-meanScores(:,:,1))./meanScores(:,:,6);
    meanScores(:,:,4) = meanScores(:,:,2)./meanScores(:,:,6);
    meanScores(:,:,2) = max(rawScores(:,:,:,2), [], 3, 'omitnan');
end

function [meanScores, rawScores] = averageScores_traverse(rawScores, allScores, robots)
    rawScores(:,:,:,1) = min(allScores(:,:,:,:,1), [], 4);
    rawScores(:,:,:,2) = max(allScores(:,:,:,:,2).*allScores(:,:,:,:,1), [], 4);
    rawScores(:,:,:,3) = max(allScores(:,:,:,:,3).*allScores(:,:,:,:,1), [], 4);
    rawScores(:,:,:,4) = sum(allScores(:,:,:,:,1), 4, 'omitnan');
    meanScores = mean(rawScores, 3, 'omitnan');
end

function cost = getCost(robot, lastRobot, i, grid)
    [~, Fnorm, Ftang, T] = quasiStaticDynamicsKnownForce(robot, i, robot.F, grid);
    Fnorm = max(Fnorm,0);
    [margin, ~, ~] = gripperMargin(Fnorm, Ftang);
    distance = robot.origin(2) - lastRobot.origin(2);
    cost = min(1, max(robot.fail, margin));
%     cost = robot.fail;
end