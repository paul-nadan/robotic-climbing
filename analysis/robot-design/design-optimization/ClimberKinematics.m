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

global X Y Z dZdx dZdy OBSTACLES FRAMES ANIMATE RECORD PLOT SIMULATE SEED_TERRAIN SEED COST_FUNCTION

% GLOBAL PARAMETERStt
SIMULATE = ~~1; % flag to run new simulation instead of using existing data
SEED_TERRAIN = ~~1; % flag to reuse a previous terrain seed
ANIMATE = ~~1; % flag to animate robot motion
RECORD = ~~0; % flag to save animations as video files
PLOT = ~~1; % flag to plot final robot condition and path
SEED = 44; % use a previous seed, -1 for same seed as last round
COST_FUNCTION = @costGripper; % e.g. costConstant, costForce, costGripper

% SIMULATION PARAMETERS
SCORES = {'Normal', 'Tangential', '|F|^2', 'Rat Mgn', 'Mag Mgn', 'Margin', 'Torque', 'Distance', 'Solve Time'}; % output variable names
CONFIG_NAMES = {'8-DoF', '9-DoF', '10-DoF', '11-DoF', '12-DoF'}; % configuration names
COLORS = {'r', [1 .5 0], [0 .7 0], 'b', [.5 0 .5]};
SWEEP = 1; % values for parameter being swept
SAMPLES = 1; % number of duplicate samples to average at each value
STEPS = 30; % number of robot steps to simulate per trial
TIME_STEP = 0.25; % delay between frame updates for animation
DISCARD_FAILS = 0; % aborts iteration for all configs if any one fails
HORIZON = 2; % number of steps solver should model (min 1)

% ROBOT CONFIGURATIONS
quads = {[2,2,2,2], [2,3,2,2], [3,3,2,2], [3,3,3,2], [3,3,3,3]};
CONFIGURATIONS = cell(size(quads));
for iConfig = 1:length(quads)
    CONFIGURATIONS{iConfig} = quadruped(quads{iConfig}, ...
        0.1, 0.3, {.2, [.16, .16]}, 0, HORIZON);
end
hex20 = hexapod(0.1, 0.3, .16, HORIZON);
CONFIGURATIONS = CONFIGURATIONS(3);

% SET VIEW WINDOW SIZE
close all;
if RECORD
    for iConfig = 1:length(CONFIGURATIONS)
        figure('units','normalized','outerposition',[0 0 1 1]);
    end
    FRAMES = cellfun(@(~) {struct('cdata',{},'colormap',{})},...
                      cell(size(CONFIGURATIONS)));
elseif ANIMATE || PLOT
    figure('units','normalized','outerposition',...
        [0 0.2, 0.3*length(CONFIGURATIONS) 0.7]);
end

% SET RANDOM SEED
if SEED_TERRAIN
    if SEED >= 0
        seed = SEED;
    end
    rng(seed);
else
    rng shuffle
    s = rng;
    seed = s.Seed;
end
fprintf('Terrain Seed: %d\n\n', seed);

% SWEEP PARAMETER
rawScores = zeros(length(SWEEP)*SAMPLES, length(CONFIGURATIONS), length(SCORES));
sumScores = zeros(length(SWEEP), length(CONFIGURATIONS), length(SCORES)+1);
if SIMULATE
    robots = cell(size(rawScores));
end
for iter = 1:size(rawScores,1)
    iSweep = ceil(iter/SAMPLES);
    sweep = SWEEP(iSweep);

    % GENERATE TERRAIN
    if SIMULATE
        [X, Y, Z, dZdx, dZdy, OBSTACLES] = terrain([-1.5, 1.5], ...
            [-1.5 3.5], .01, [1,1,0.5]*sweep, [1, .25, 0.0625], 0);
    end
    
    % INITIALIZE ROBOT
    for iConfig = 1:length(CONFIGURATIONS)
        if ~SIMULATE
            break
        end
        robot = spawnRobot([0;-.75;0], eye(3), CONFIGURATIONS{iConfig});
        robots{iter, iConfig} = repmat(robot, STEPS + 1, 1);
    end
    
    % RUN SIMULATION
    aborted = 0;
    stepScores = zeros(STEPS, length(CONFIGURATIONS), length(SCORES));
    fprintf('Scores:  [');
    fprintf('%10s, ', string(SCORES));
    fprintf(']\n');
    for i = 1:STEPS
        if i < 10
            fprintf('Step %d:  ', i);
        else
            fprintf('Step %d: ', i);
        end
        for iConfig = 1:length(CONFIGURATIONS)
            dt = NaN;
            lastRobot = robots{iter, iConfig}(i);
            if lastRobot.fail
                robot = lastRobot;
                robots{iter, iConfig}(i+1) = robot;
            elseif SIMULATE
                tic();
                robot = step(lastRobot, i);
                dt = toc();
                robots{iter, iConfig}(i+1) = robot;
            else
                robot = robots{iter, iConfig}(i+1);
            end
            if robot.fail
                stepScores(i,iConfig,:) = NaN;
                fprintf('Failed to converge\n');
            else
                stepScores(i,iConfig,:) = [computeScores(robot, lastRobot, i), dt];
                fprintf('[');
                fprintf('%10.3f, ', stepScores(i,iConfig,:));
                fprintf(']\n');
            end
            if RECORD
                figure(iConfig);
            elseif ANIMATE
                subplot(1, length(CONFIGURATIONS), iConfig);
                title(CONFIG_NAMES{iConfig});
            end
            animateStep(lastRobot, robot, TIME_STEP, i, iConfig);
            if DISCARD_FAILS && robot.fail
                aborted = 1;
                fprintf('Failed on configuration: %d\n', iConfig);
                break
            end
        end
        if aborted
            fprintf('Aborted\n');
            break
        end
    end
    
    % EVALUATE ITERATION RESULTS
    if ~aborted
        for iConfig = 1:length(CONFIGURATIONS)
            for iScore = 1:size(stepScores, 3)
                rawScores(iter, iConfig, iScore) = mean(stepScores(:, iConfig, iScore), 'omitnan');
            end
            
            % Ignore individual failed samples
%             if robot.fail
%                 rawScores(iter, iConfig, :) = 0;
%                 sumScores(iSweep, iConfig, end) = sumScores(iSweep, iConfig, end)-1;
%             end
            robot = robots{iter, iConfig}(end);
            [F1, ~, ~, T] = quasiStaticDynamicsKnownForce(robot, STEPS, robot.F);
            F2 = quasiStaticDynamics(robot, STEPS);
            path = [robots{iter, iConfig}.origin];
            % Plot final robot state
            if PLOT
                if RECORD
                    figure(iConfig);
                else
                    subplot(1, length(CONFIGURATIONS), iConfig);
                end
                plotTerrain();
                plotRobot(robots{iter, iConfig}(end));
                plotForces(robot, F2, STEPS, 'r', 0.016);
                plotForces(robot, F1, STEPS, 'g', 0.016);
%                 plotTorques(robot, T, 'c', 0.1);
                plot3(path(1,:), -path(3,:), path(2,:),'k','linewidth', 2);
                title(CONFIG_NAMES{iConfig});
                drawnow();
            end
            
            % Print iteration results
            fprintf('Average: [');
            fprintf('%10.3f, ', rawScores(iter, iConfig, :));
            fprintf(']\n');
            fprintf('Sweep: %.3f, Configuration: %d, Sample: %d\n\n', sweep, iConfig, mod(iter, SAMPLES));
        end
        sumScores(iSweep, :, :) = sumScores(iSweep, :, :) + ...
            cat(3, rawScores(iter, :, :), ones(1,length(CONFIGURATIONS)));
    end
end

% EVALUATE CUMULATIVE RESULTS
meanScores = sumScores(:, :, 1:end-1)./sumScores(:, :, end);
if size(meanScores,1) > 1
    if length(CONFIGURATIONS) > 1
        figure('units','normalized','outerposition',...
        [0 0.2, 0.3*length(SCORES) 0.45]);
    else
        figure();
    end
    
    % If multiple configs and scores, use separate figures for each score
    for score = 1:length(SCORES)
        if length(CONFIGURATIONS) > 1
            subplot(1, length(SCORES), score);
        end
        for iConfig = 1:length(CONFIGURATIONS)
            c = COLORS{score};
            if length(CONFIGURATIONS) > 1
                c = COLORS{iConfig};
            end
            plot(SWEEP, meanScores(:,iConfig,score), 'color', c, 'linewidth', 3);
            hold on;
        end
        xlabel('Terrain Difficulty');
        ylabel('Force (N) / Torque (N-m)');
        if length(CONFIGURATIONS) > 1
            legend(CONFIG_NAMES, 'Location','northwest');
            title(SCORES{score});
        elseif score == length(SCORES)
            legend(SCORES, 'Location','northwest');
            title(CONFIG_NAMES{1});
        end
    end
end

% Plot margin vs step for 1 trial
marginRat = stepScores(:,1,4);
marginMag = stepScores(:,1,5);
margin = stepScores(:,1,6);
figure(); hold on;
plot(marginRat, 'r');
plot(marginMag, 'b');
plot(margin, 'k');
plot(1+0*margin, 'k--')
xlabel('Step'); ylabel('Margin');
legend('Ratio Margin', 'Magnitude Margin', 'Combined Margin', 'Maximum Threshold');
title('Grasp Adhesion Margin');

function scores = computeScores(robot, lastRobot, i)
    global COST_FUNCTION
    [F1, n1,t1,~] = quasiStaticDynamicsMargin(robot, i);
    [F2, ~,~,~] = quasiStaticDynamicsKnownForce(robot, i, robot.F);
    margin1 = gripperMargin(n1, t1);
    deltaForce = sum(abs(F1-F2), 'all');
    fprintf('Force difference = %f, ', deltaForce);
    if isequal(func2str(COST_FUNCTION), 'costConstant')
        [~, Fnorm, Ftang, T] = quasiStaticDynamics(robot, i);
    else
        [~, Fnorm, Ftang, T] = quasiStaticDynamicsKnownForce(robot, i, robot.F);
    end
    normalForce = max([Fnorm, 0]);
    tangentForce = max(Ftang);
    torque = max(vecnorm(T));
    Fnorm = max(Fnorm,0);
    Fmag = sqrt(Fnorm.^2 + Ftang.^2);
    [margin, ratioMargin, magnitudeMargin] = gripperMargin(Fnorm, Ftang);
    fprintf('Margin improvement = %f\n', margin-margin1);
    ratio = normalForce./tangentForce;
%     pathLength = norm(robot.origin - lastRobot.origin);
    distance = robot.origin(2) - lastRobot.origin(2);
    scores = [normalForce, tangentForce, sum(Fmag.*Fmag), max(ratioMargin), max(magnitudeMargin), margin, torque, distance];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TERRAIN GENERATION FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generates random terrain composed of planar segments
function [xq, yq, zq, dzdx, dzdy, obstacles] = terrain(x, y, res, slope, roughness, corner)
    % Fine height grid
    [xq,yq] = meshgrid(x(1):res:x(2), y(1):res:y(2));
    zq = zeros(size(xq));
    for i = 1:length(slope)
        % Sparse height grid
        [X,Y] = meshgrid(x(1):roughness(i):x(2), y(1):roughness(i):y(2));
        z = (slope(i)*roughness(i))*rand(size(X));
        zq = zq + griddata(X,Y,z,xq,yq);
    end
    
    zq = zq + corner*yq.*(yq>0) - corner*yq.*(yq<0);
    obstacles = [];%[[-Inf, Inf; 0, 0.2; -Inf, 0.4],[-Inf, Inf; -0.2, 0; -Inf, 0.25],[-Inf, Inf; 0.2, 0.4; -Inf, 0.25]];
    
    % Slopes
%     figure(); plot(xq(1, 1:end-1), zq(1, 1:end-1)); title('Unsmoothed'); xlabel('x'); ylabel('z');
%     zq = imgaussfilt(zq, 2);
    dzdy = diff(zq)/res;
    dzdx = diff(zq')'/res;
%     figure(); plot(xq(1, 1:end-1), dzdx(1,:)); title('Unsmoothed'); xlabel('x'); ylabel('dzdx');
%     zq = imgaussfilt(zq, 2);
%     dzdx = diff(zq')'/res;
%     figure(); plot(xq(1, 1:end-1), zq(1, 1:end-1)); title('Smoothed'); xlabel('x'); ylabel('z');
%     figure(); plot(xq(1, 1:end-1), dzdx(1,:)); title('Smoothed'); xlabel('x'); ylabel('dzdx');
    % Trim off-by-one indices
    xq = xq(1:end-1, 1:end-1);
    yq = yq(1:end-1, 1:end-1);
    zq = zq(1:end-1, 1:end-1);
    dzdx = dzdx(1:end-1, :);
    dzdy = dzdy(:, 1:end-1);
end

% Find value of function Z at (x,y) by interpolation
function z = f(x, y, Z)
    global X Y
    if x > max(X(1,:))
        x = max(X(1,:));
    elseif x < min(X(1,:))
        x = min(X(1,:));
    end
    if y > max(Y(:,1))
        y = max(Y(:,1));
    elseif y < min(Y(:,1))
        y = min(Y(:,1));
    end
    dx = X(2,2)-X(1,1);
    x0 = x-X(1,1);
    y0 = y-Y(1,1);    
    i1 = floor(1+x0/dx);
    j1 = floor(1+y0/dx);
    i2 = ceil(1+x0/dx);
    j2 = ceil(1+y0/dx);
    c = Z(sub2ind(size(Z), [j1; j1; j2; j2], [i1; i2; i1; i2]));
    xp = x0/dx - i1 + 1;
    yp = y0/dx - j1 + 1;
    y1 = c(1,:).*(1-xp) + c(2,:).*xp;
    y2 = c(3,:).*(1-xp) + c(4,:).*xp;
    z = y1.*(1-yp) + y2.*yp;
end

% Returns distance to nearest collision with terrain
function distance = contact(X)
    global Z OBSTACLES
    gDist = (X(3,:)-f(X(1,:), X(2,:), Z))';
    distance = gDist;
    for i = 1:size(OBSTACLES, 2)/2
        obstacle = OBSTACLES(:, 2*i-1:2*i);
        oDist = max(max(obstacle(:,1) - X(:,:), X(:,:) - obstacle(:,2)))';
        distance = min(distance, oDist);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% VISUALIZATION FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plotTerrain()
    global X Y Z OBSTACLES
    cla();
    zq = Z;
    for i = 1:size(OBSTACLES, 2)/2
        obstacle = OBSTACLES(:, 2*i-1:2*i);
        iObstacle = (X>=obstacle(1,1)&X<=obstacle(1,2))&...
            (Y>=obstacle(2,1)&Y<=obstacle(2,2));
        zq(iObstacle) = obstacle(3,2);
    end
    mesh(X(1:2:end,1:2:end),-zq(1:2:end,1:2:end),Y(1:2:end,1:2:end), zq(1:2:end,1:2:end));
    hold on
    axis equal
end

function plotRobot(r)
    global PLOT
    if ~PLOT
        return
    end
    hold on;
    for iBody = 1:length(r.bodies)
        body = r.bodies{iBody};
        fill3(body(1,:),-body(3,:), body(2,:), 'r');
    end
    plotPoints(r.vertices(:,sum(r.gait.feet, 2)>0), 'b.');
%     plotPoints(r.feet(:,~r.c), 'r.');
    for i = 1:size(r.vertices,2)
       plotLine(r.vertices(:,i), r.vertices(:,i)-r.links(:,i), 'b');
    end
end

function plotForces(robot, F, count, color, scale)
    i = mod(count, size(robot.gait.angles, 2))+1;
    feet = robot.vertices(:, robot.gait.feet(:,i) > 0);
    for i = 1:size(F, 2)-1
        foot = feet(:, i);
        quiver3(foot(1), -foot(3), foot(2), F(1, i)*scale, ...
            -F(3, i)*scale, F(2, i)*scale, color, 'linewidth', 2);
    end
    c = robot.origin;
    quiver3(c(1), -c(3), c(2), F(1, end)*scale, -F(3, end)*scale, ...
        F(2, end)*scale, color, 'linewidth', 2);
end

function plotTorques(robot, T, color, scale)
    joints = robot.vertices - robot.links;
    for i = 1:size(T, 2)-1
        joint = joints(:, i);
        quiver3(joint(1), -joint(3), joint(2), T(1, i)*scale, ...
            -T(3, i)*scale, T(2, i)*scale, color, 'linewidth', 2);
    end
end

function g = plotLine(p1, p2, c)
     g = plot3([p1(1), p2(1)],...
          -[p1(3), p2(3)],...
           [p1(2), p2(2)], c, 'linewidth', 2);
end

function g = plotPoints(p, c)
     g = plot3(p(1,:),-p(3,:), p(2,:), c, 'markersize', 20);
end

function animateStep(r1, r2, dt, count, window)
    i = mod(count-1, size(r1.gait.angles, 2))+1;
    global FRAMES RECORD ANIMATE
    if ~ANIMATE && ~RECORD
        return
    end
    dBody = cell(1,length(r1.bodies));
    for iBody = 1:length(r1.bodies)
        dBody{iBody} = r2.bodies{iBody} - r1.bodies{iBody};
    end
    dJoints = (r2.vertices-r2.links) - (r1.vertices-r1.links);
    dVertices = r2.vertices - r1.vertices;
    plotTerrain();
    plotPoints(r1.vertices(:,r1.gait.feet(:,i)>0), 'b.');
    for t = 0:dt:1
        for iBody = 1:length(r1.bodies)
            body = r1.bodies{iBody}+t*dBody{iBody};
            g.bodies(iBody) = fill3(body(1,:), -body(3,:), body(2,:), 'r');
        end
        vertices = r1.vertices + dVertices*t;
        joints = r1.vertices - r1.links + dJoints*t;
        for iVertex = 1:size(r1.vertices,2)
            g.links(iVertex) = plotLine(joints(:,iVertex),...
                vertices(:,iVertex), 'b');
        end
        drawnow();
        if RECORD
            FRAMES{window}(length(FRAMES{window})+1) = getframe;
        end
        if t ~= 1
            for iBody = 1:length(g.bodies)
                delete(g.bodies(iBody));
            end
            for iLink = 1:length(g.links)
                delete(g.links(iLink));
            end
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROBOT KINEMATICS FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Creates a new robot in an initial state
function robot = spawnRobot(origin, R0, config)
    global Z
    robot = getRobot(origin, R0, config.gait.angles(:, end), config);
    body = [robot.bodies{:}];
    dz = config.clearance + f(body(1, :), body(2, :), Z) - body(3, :)+0.1;
    robot = move(robot, [0;0;max(dz)], eye(3));
    robot = step(robot, 0);
end

% Returns a robot with the given state and configuration
function robot = getRobot(origin, R0, angles, config)
    robot.origin = origin;
    robot.R0 = R0;
    robot.angles = angles;
    robot.config = config;
    robot.gait = config.gait;
    robot.R = zeros(3,3,length(angles));
    robot.links = zeros(3, length(angles));
    robot.vertices = zeros(3, length(angles));
    for iLink = 1:length(angles)
        robot.R(:,:,iLink) = vrrotvec2mat([config.joints(:,2,iLink); ...
            deg2rad(angles(iLink))]);
        if config.parents(iLink) == 0
            Rp = R0;
            Xp = origin;
        else
            Rp = robot.R(:,:,config.parents(iLink));
            Xp = robot.vertices(:, config.parents(iLink));
        end
        robot.R(:,:,iLink) = Rp*robot.R(:,:,iLink);
        robot.links(:,iLink) = robot.R(:,:,iLink)*config.joints(:,3,iLink);
        robot.vertices(:,iLink) = Xp + Rp*config.joints(:,1,iLink)+...
            robot.links(:,iLink);
    end
    robot.bodies = {};
    for i = 1:length(config.bodies)
        if config.iBodies(i)
            centroid = robot.vertices(:,config.iBodies(i));
            robot.bodies{i} = centroid + ...
                robot.R(:,:,config.iBodies(i))*config.bodies{i};
        else
            robot.bodies{i} = origin + ...
                R0*config.bodies{i};
        end
    end
end

% Updates robot joint positions after a change in state variables
function robot = update(robot)
    robot = getRobot(robot.origin, robot.R0, robot.angles, robot.config);
end

% Creates a robot with the given state vector and configuration
function robot = state2robot(state, config)
    B = cross(state(4:6), state(7:9));
    R0 = [-state(7:9), state(4:6), B];
    robot = getRobot(state(1:3), R0, state(10:end), config);
end

% Returns the state vector describing a given robot
function state = robot2state(robot)
    T0 = robot.R0(:,2);
    N0 = -robot.R0(:,1);
    state = [robot.origin; T0; N0; robot.angles];
end

% Translates or rotates a robot as a rigid body
function robot = move(robot, dx, dR)
    robot.links = dR*robot.links;
    robot.vertices = dR*(robot.vertices-robot.origin) + robot.origin;
    for iBody = 1:length(robot.bodies)
        robot.bodies{iBody} = dR*(robot.bodies{iBody}-robot.origin)+...
            robot.origin;
        robot.bodies{iBody} = robot.bodies{iBody} + dx;
    end
    robot.origin = robot.origin + dx;
    robot.vertices = robot.vertices + dx;
    robot.R0 = dR*robot.R0;
    for iR = 1:size(robot.R, 3)
        robot.R(:,:,iR) = dR*robot.R(:,:,iR);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% KINEMATIC SOLVER FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function r = step(r0, count0)
    global COST_FUNCTION
    horizon = r0.gait.horizon;
    count = count0:count0+horizon-1;
    i = mod(count-1, size(r0.gait.angles, 2))+1;
    targetHeading = 2*r0.origin(1);
    headingVec = r0.R0(:,2);
    heading = atan2(-headingVec(1), headingVec(2));
    yawErr = max(-r0.gait.dyaw, min(r0.gait.dyaw, targetHeading-heading));
    r0.R0 = vrrotvec2mat([0 0 1 yawErr])*r0.R0;
    rd = repmat(r0, horizon+1, 1);
    X0 = robot2state(rd(2));
    [F0,~,~,~] = quasiStaticDynamics(rd(2),count0);
    F0 = reshape(F0(:,1:end-1), [], 1);
    Xn = zeros(length(X0)*horizon, 1);
    Aeq = zeros(horizon, length(Xn)+length(F0));
    delta = norm(r0.gait.dx(1:2,i));
    lb = repmat([r0.origin(1:2)-delta;-Inf;-1;-1;-1;-1;-1;-1;r0.config.limits(:,1)], horizon, 1);
    ub = repmat([r0.origin(1:2)+delta;Inf;1;1;1;1;1;1;r0.config.limits(:,2)], horizon, 1);
    for iStep = 1:horizon
        rd(iStep+1).origin = rd(iStep).origin + rd(iStep).R0*r0.gait.dx(:,i(iStep));
        rd(iStep+1).angles = r0.gait.angles(:,i(iStep));
        ix0 = length(X0)*(iStep-1);
        Xn(ix0+1:ix0+length(X0)) = robot2state(rd(iStep+1));
        Aeq(iStep, (iStep-1)*length(X0)+4:(iStep-1)*length(X0)+6) = X0(7:9)';
        lb(ix0+1:ix0+2) = rd(iStep+1).origin(1:2)-delta;
        ub(ix0+1:ix0+2) = rd(iStep+1).origin(1:2)+delta;
    end
    if isfield(r0, 'seed')
        h = min(horizon*length(X0), length(r0.seed));
        Xn(1:h) = r0.seed(1:h);
    end
    [F0,~,~,~] = quasiStaticDynamics(state2robot(Xn(1:length(X0)),r0.config), count0);
    F0 = reshape(F0(:,1:end-1), [], 1);
    lb = [lb; zeros(size(F0))-Inf];
    ub = [ub; zeros(size(F0))+Inf];
    beq = zeros(horizon, 1);
    options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
        'Algorithm','sqp','ConstraintTolerance',1e-4,...
        'Display','iter','SpecifyObjectiveGradient',false, 'CheckGradients', false, 'FiniteDifferenceType', 'central');
    
    % DEBUG
%     [c0, ceq0] = constraintsLookAhead([Xn;F0],rd,i,count0>0)
%     stateVectorBounds = [lb, [Xn;F0], ub]
    % First find any valid solution
    [xFree,~,~,outputFree] = fmincon(@(x)costConstant(x,rd,i),[Xn;F0],[],[],...
        Aeq,beq,lb,ub,@(x)constraintsLookAhead(x,rd,i,count0>0),options);
    Xn = xFree(1:length(Xn));
    robotFree = state2robot(Xn(1:length(X0)), r0.config);
    [F0,~,~,~] = quasiStaticDynamicsMargin(robotFree,count0);
    F0 = reshape(F0(:,1:end-1), [], 1);
    
    [x,~,~,output] = fmincon(@(x)COST_FUNCTION(x,rd,i),[Xn;F0],[],[],...
        Aeq,beq,lb,ub,@(x)constraintsLookAhead(x,rd,i,count0>0),options);
    % fallback
    if count0>0&&output.constrviolation > options.ConstraintTolerance
        fprintf('Falling back on no-cost solution\n');
        x = [Xn; F0];
        output = outputFree;
    end
    
    r = state2robot(x(1:length(X0)), r0.config);
    r.F = reshape(x(end-length(F0)+1:end), 3, []);
    r.fail = count0>0&&output.constrviolation > options.ConstraintTolerance;
    if r.fail
        fprintf('\n');
        disp(output.message);
        r.seed = r0.seed;
        if horizon > 1
            fprintf('Trying shorter horizon: %d\n', horizon-1);
            r0.R0 = vrrotvec2mat([0 0 1 -yawErr])*r0.R0;
            r0.gait.horizon = r0.gait.horizon - 1;
            r = step(r0, count0);
            r.gait.horizon = horizon;
        end
    else
        r.seed = x(length(X0)+1:horizon*length(X0));
    end
end

% Gripper adhesion metric
function [c, g] = costGripper(x, rd, i)
    global dZdx dZdy
    iF = mod(i(1), size(rd(1).gait.angles, 2))+1; % next step in gait cycle
    nF = 3*sum(rd(2).gait.feet(:,iF) > 0); % number of force variables
    xF = x(end-nF+1:end); % force components
    Nx = (length(x)-nF)/(length(rd)-1); % length of robot state vector
    robot = state2robot(x(1:Nx), rd(1).config);
    feet = robot.vertices(:, robot.gait.feet(:,iF) > 0);
    
    F = reshape(xF, 3, []);
    
    % Find normal vectors
    N = zeros(3,size(feet, 2));
    for iFoot = 1:size(feet, 2)
        foot = feet(:,iFoot);
        N(:,iFoot) = [-f(foot(1),foot(2),dZdx);...
                       -f(foot(1),foot(2),dZdy); 1];
        N(:,iFoot) = N(:,iFoot)/norm(N(:,iFoot));
    end
    
    % Decompose forces into components
    Fnorm = zeros(1,size(feet,2));
    Ftang = zeros(1,size(feet,2));
    for iFoot = 1:size(feet,2)
        Fnorm(iFoot) = -F(:,iFoot)'*N(:,iFoot);
        Ftang(iFoot) = norm(cross(F(:,iFoot),N(:,iFoot)));
        Fnorm(iFoot) = max(0, Fnorm(iFoot));
    end
    
    % Compute margin
    c = gripperMargin(Fnorm, Ftang);
    g = zeros(size(x));
end

function [c, c1, c2] = gripperMargin(Fnorm, Ftang)
    Fmax = 25;
    Fnorm = max(Fnorm, 0);
    c1 = Fnorm./max(Ftang, 1e-6)/tand(20);
    c2 = sqrt(Fnorm.^2+Ftang.^2)./Fmax;
    c = max([c1, c2]);
end

% Magnitude of the total contact forces (F'*F)
function [c, g] = costForce(x, rd, i)
    iF = mod(i(1), size(rd(1).gait.angles, 2))+1; % next step in gait cycle
    nF = 3*sum(rd(2).gait.feet(:,iF) > 0); % number of force variables
    xF = x(end-nF+1:end); % force components
    c = xF'*xF;
    g = [zeros(length(x)-nF, 1); 2*xF];
end

% Set cost to zero and focus on satisfying constraints
function [c, g] = costConstant(x, rd, i)
    c = 0;
    g = zeros(size(x));
end

% Center of mass position error from default gait
function [c, g] = costKinematic(x, r0, ~)
    d = r0.origin(1:2) - x(1:2);
    c = max(norm(d),1e-12);
    g = [d(1)/c;
         d(2)/c;
         zeros(length(x)-2,1)];
end

function [c,ceq] = constraintsLookAhead(x, rd, i, stepping)
    iF = mod(i(1), size(rd(1).gait.angles, 2))+1; % next step in gait cycle
    nF = 3*sum(rd(2).gait.feet(:,iF) > 0); % number of force variables
    xF = x(end-nF+1:end);
    Nx = (length(x)-nF)/(length(rd)-1);
    [cF, ceqF] = constraintsForce(xF, state2robot(x(1:Nx), rd(2).config), iF);
    [c1,ceq1] = constraints(x(1:Nx), rd(2), i(1), stepping);
    ceq = zeros(length(ceq1)*(length(rd)-1), 1);
    c = zeros(length(c1)*(length(rd)-1), 1);
    ceq(1:length(ceq1)) = ceq1;
    c(1:length(c1)) = c1;
    for iStep = 2:length(rd)-1
        r = state2robot(x(Nx*(iStep-2)+1:Nx*(iStep-1)), rd(1).config);
        [cn,ceqn] = constraints(x(Nx*(iStep-1)+1:Nx*iStep), r, i(iStep), 1);
        ceq(length(ceqn)*(iStep-1)+1:length(ceqn)*iStep) = ceqn;
        c(length(cn)*(iStep-1)+1:length(cn)*iStep) = cn;
    end
    c = [c; cF];
    ceq = [ceq; ceqF];
    % DEBUG
%     if abs(x(1) - 0.0459) < 0.0001
%         if length(rd) == 2
%             the_state = x'
%             the_c = c'
%             the_ceq = ceq'
%         end
%     end
end

% x = current state, r0 = initial configuration (for stance feet)
function [c,ceq] = constraints(x, r0, i, stepping)
    r = state2robot(x, r0.config);
    allFeet = r.vertices(:, sum(r0.gait.feet, 2) > 0);
    stanceFeet = r.vertices(:, r0.gait.feet(:,i) > 0);
    fixedFeet = r0.vertices(:, r0.gait.feet(:,i) > 0);
    ceq = [x(4:6)'*x(7:9); %1
           norm(x(4:6))-1; %1
           norm(x(7:9))-1; %1
           contact(allFeet); %4
           stepping*(stanceFeet(1,:)-fixedFeet(1,:))'; %2
           stepping*(stanceFeet(2,:)-fixedFeet(2,:))']; %2
    body = [r.bodies{:}];
    dz = -contact(body) + r0.config.clearance;
    B = cross(x(4:6), x(7:9));
    c = [dz; -B(3)];
end

function [c,ceq] = constraintsForce(xF, robot, iF)    
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    WEIGHT = 5*9.81; % Magnitude of gravity force (N)
    feet = robot.vertices(:, robot.gait.feet(:,iF) > 0);
    r = feet - robot.origin;
    
    % Find contact forces
    G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*WEIGHT;
    Aeq_torque = zeros(3,3*size(feet,2));
    for iFoot = 1:size(feet,2)
        ri = r(:,iFoot);
        Aeq_torque(:,iFoot*3-2:iFoot*3) = [0, ri(3), -ri(2);
                                           -ri(3), 0, ri(1);
                                           ri(2), -ri(1), 0];
    end
    Aeq = [repmat(eye(3), 1, size(feet,2)); Aeq_torque];
    beq = [-G;zeros(3,1)];
    ceq = Aeq*xF - beq;
    c = [];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% QUASI-STATIC DYNAMICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Find contact forces that optimize force magnitude
function [F, Fnorm, Ftang, T] = quasiStaticDynamics(robot, count)
    global dZdx dZdy
    
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    WEIGHT = 5*9.81; % Magnitude of gravity force (N)
    
    i = mod(count, size(robot.gait.angles, 2))+1;
    feet = robot.vertices(:, robot.gait.feet(:,i) > 0);
    r = feet - robot.origin;
    
    % Find normal vectors
    N = zeros(3,size(feet, 2));
    for iFoot = 1:size(feet, 2)
        foot = feet(:,iFoot);
        N(:,iFoot) = [-f(foot(1),foot(2),dZdx);...
                       -f(foot(1),foot(2),dZdy); 1];
        N(:,iFoot) = N(:,iFoot)/norm(N(:,iFoot));
    end
    
    % Find contact forces
    G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*WEIGHT;
    Aeq_torque = zeros(3,3*size(feet,2));
    for iFoot = 1:size(feet,2)
        ri = r(:,iFoot);
        Aeq_torque(:,iFoot*3-2:iFoot*3) = [0, ri(3), -ri(2);
                                           -ri(3), 0, ri(1);
                                           ri(2), -ri(1), 0];
    end
    Aeq = [repmat(eye(3), 1, size(feet,2)); Aeq_torque];
    beq = [-G;zeros(3,1)];
    A = [];
    b = [];
    H = eye(3*size(feet,2));
    h = zeros(size(H,1), 1);
%     h = reshape(N, [], 1);
    options = optimoptions('quadprog','Display','off');
    [Fvec,~,~,~] = quadprog(H,h,A,b,Aeq,beq,[],[],[],options);
    F = [reshape(Fvec, 3, []), G];
    
    % Decompose forces into components
    Fnorm = zeros(1,size(feet,2));
    Ftang = zeros(1,size(feet,2));
    for iFoot = 1:size(feet,2)
        Fnorm(iFoot) = -F(:,iFoot)'*N(:,iFoot);
        Ftang(iFoot) = norm(cross(F(:,iFoot),N(:,iFoot)));
    end
    
    % Compute torques
    joints = robot.vertices - robot.links;
    T = zeros(size(joints));
    for iJoint = 1:size(robot.vertices, 2)
        nChildren = find([robot.config.parents(iJoint+1:end),0]<iJoint, 1);
        iChildren = iJoint:iJoint+nChildren-1;
        iFeetBool = double(robot.gait.feet(:,i)' > 0);
        iFeetBool(iChildren) = -iFeetBool(iChildren);
        iFeet = find(iFeetBool(iFeetBool~=0) < 0);
        for iFoot = iFeet
            rFoot = feet(:,iFoot)-joints(:,iJoint);
            T(:, iJoint) = T(:, iJoint) + cross(rFoot, F(:,iFoot));
        end
    end
end

% Find contact forces that optimize adhesion margin
function [F, Fnorm, Ftang, T] = quasiStaticDynamicsMargin(robot, count)
    global dZdx dZdy
    
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    WEIGHT = 5*9.81; % Magnitude of gravity force (N)
    
    i = mod(count, size(robot.gait.angles, 2))+1;
    feet = robot.vertices(:, robot.gait.feet(:,i) > 0);
    r = feet - robot.origin;
    
    % Find normal vectors
    N = zeros(3,size(feet, 2));
    for iFoot = 1:size(feet, 2)
        foot = feet(:,iFoot);
        N(:,iFoot) = [-f(foot(1),foot(2),dZdx);...
                       -f(foot(1),foot(2),dZdy); 1];
        N(:,iFoot) = N(:,iFoot)/norm(N(:,iFoot));
    end
    
    % Find contact forces
    G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*WEIGHT;
    Aeq_torque = zeros(3,3*size(feet,2));
    for iFoot = 1:size(feet,2)
        ri = r(:,iFoot);
        Aeq_torque(:,iFoot*3-2:iFoot*3) = [0, ri(3), -ri(2);
                                           -ri(3), 0, ri(1);
                                           ri(2), -ri(1), 0];
    end
    X0 = robot2state(robot);
    Aeq = [repmat(eye(3), 1, size(feet,2)); Aeq_torque];
    Aeq = [zeros(6,length(X0)), Aeq];
    beq = [-G;zeros(3,1)];
    [F0,~,~,~] = quasiStaticDynamics(robot, count);
    F0 = reshape(F0(:,1:end-1), [], 1);
    lb = [X0; -ones(size(F0))*Inf];
    ub = [X0; ones(size(F0))*Inf];
    rd = [robot, robot];
    options = optimoptions('fmincon','Algorithm','sqp','Display','none');
    [X,~,~,~] = fmincon(@(x)costGripper(x, rd, count),[X0;F0],[],[],...
        Aeq,beq,lb,ub,[],options);
    Fvec = X(length(X0)+1:end);
    F = [reshape(Fvec, 3, []), G];
    
    % Decompose forces into components
    Fnorm = zeros(1,size(feet,2));
    Ftang = zeros(1,size(feet,2));
    for iFoot = 1:size(feet,2)
        Fnorm(iFoot) = -F(:,iFoot)'*N(:,iFoot);
        Ftang(iFoot) = norm(cross(F(:,iFoot),N(:,iFoot)));
    end
    
    % Compute torques
    joints = robot.vertices - robot.links;
    T = zeros(size(joints));
    for iJoint = 1:size(robot.vertices, 2)
        nChildren = find([robot.config.parents(iJoint+1:end),0]<iJoint, 1);
        iChildren = iJoint:iJoint+nChildren-1;
        iFeetBool = double(robot.gait.feet(:,i)' > 0);
        iFeetBool(iChildren) = -iFeetBool(iChildren);
        iFeet = find(iFeetBool(iFeetBool~=0) < 0);
        for iFoot = iFeet
            rFoot = feet(:,iFoot)-joints(:,iJoint);
            T(:, iJoint) = T(:, iJoint) + cross(rFoot, F(:,iFoot));
        end
    end
end

% Compute force components and torques from contact forces
function [F, Fnorm, Ftang, T] = quasiStaticDynamicsKnownForce(robot, count, F)
    global dZdx dZdy
    
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    WEIGHT = 5*9.81; % Magnitude of gravity force (N)
    
    i = mod(count, size(robot.gait.angles, 2))+1;
    feet = robot.vertices(:, robot.gait.feet(:,i) > 0);
    r = feet - robot.origin;
    
    % Find normal vectors
    N = zeros(3,size(feet, 2));
    for iFoot = 1:size(feet, 2)
        foot = feet(:,iFoot);
        N(:,iFoot) = [-f(foot(1),foot(2),dZdx);...
                       -f(foot(1),foot(2),dZdy); 1];
        N(:,iFoot) = N(:,iFoot)/norm(N(:,iFoot));
    end
    
    % Find contact forces
    G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*WEIGHT;
    Fvec = reshape(F, [], 1);
    F = [reshape(Fvec, 3, []), G];
    
    % Validate equilibrium
    Aeq_torque = zeros(3,3*size(feet,2));
    for iFoot = 1:size(feet,2)
        ri = r(:,iFoot);
        Aeq_torque(:,iFoot*3-2:iFoot*3) = [0, ri(3), -ri(2);
                                           -ri(3), 0, ri(1);
                                           ri(2), -ri(1), 0];
    end
    Aeq = [repmat(eye(3), 1, size(feet,2)); Aeq_torque];
    beq = [-G;zeros(3,1)];
    equilibriumError = max(abs(Aeq*Fvec-beq))/max(abs(beq));
    if equilibriumError > 1e-6
        fprintf('Forces violate equilibrium constraint: error = %.3e\n', equilibriumError);
    end
    
    % Decompose forces into components
    Fnorm = zeros(1,size(feet,2));
    Ftang = zeros(1,size(feet,2));
    for iFoot = 1:size(feet,2)
        Fnorm(iFoot) = -F(:,iFoot)'*N(:,iFoot);
        Ftang(iFoot) = norm(cross(F(:,iFoot),N(:,iFoot)));
    end
    
    % Compute torques
    joints = robot.vertices - robot.links;
    T = zeros(size(joints));
    for iJoint = 1:size(robot.vertices, 2)
        nChildren = find([robot.config.parents(iJoint+1:end),0]<iJoint, 1);
        iChildren = iJoint:iJoint+nChildren-1;
        iFeetBool = double(robot.gait.feet(:,i)' > 0);
        iFeetBool(iChildren) = -iFeetBool(iChildren);
        iFeet = find(iFeetBool(iFeetBool~=0) < 0);
        for iFoot = iFeet
            rFoot = feet(:,iFoot)-joints(:,iJoint);
            T(:, iJoint) = T(:, iJoint) + cross(rFoot, F(:,iFoot));
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATION GENERATOR FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Helper function for initializing configurations
function parents = findParents(count)
    parents = zeros(1, sum(count));
    for i = 1:length(parents)
        p = find(count(1:i), 1, 'last');
        count(p) = count(p) - 1;
        parents(i) = p-1;
    end
end

% Four-legged robot with given leg DoF and either trot or quasistatic gait
function config = quadruped(dof, w, h, L, trot, horizon)
    config.count = [4, ones(1,dof(1)-1), 0, ones(1,dof(2)-1), 0,...
                       ones(1,dof(3)-1), 0, ones(1,dof(4)-1), 0];
    config.clearance = 0.04;
    config.bodies = {[-w/2,w/2,w/2,-w/2; h/2,h/2,-h/2,-h/2; 0,0,0,0]};
    config.iBodies = 0; % indices of joint for each body segment
    config.joints = zeros(3, 3, sum(config.count));
    config.limits = zeros(sum(config.count), 2); % angle limits
    config.gait.dyaw = deg2rad(5); % gait max yaw correction per step
    config.gait.horizon = horizon;
    if trot
        config.gait.angles = zeros(sum(config.count), 2); % gait states
        config.gait.feet = zeros(sum(config.count), 2); % foot contacts
        config.gait.dx = [0 0; 0.1 0.1; 0 0]; % gait centroid motion
    else
        config.gait.angles = zeros(sum(config.count), 4); % gait states
        config.gait.feet = zeros(sum(config.count), 4); % foot contacts
        config.gait.dx = 0.05*[0 0 0 0; 1 1 1 1; 0 0 0 0];
    end
    x = [1;0;0]; % right
    y = [0;1;0]; % forward
    z = [0;0;1]; % up
    gait = [-1, 1, -1, 1]*40;
    feet = [1, 0, 1, 0];
    feet2 = [1 3 2 4];
    gait2 = [3/4, 1/4, -1/4, -3/4]*40;
    for iLeg = 1:4
        iJoint = sum(dof(1:iLeg-1))+1;
        config.joints(:,1,iJoint) = config.bodies{1}(:,iLeg); % shoulder
        config.joints(:,2,iJoint) = z; % shoulder yaw axis
        config.joints(:,2,iJoint+1) = y; % shoulder roll axis
        config.joints(:,3,iJoint+1) = x*L{dof(iLeg)-1}(1); % link 1
        config.limits(iJoint:iJoint+1,:) = [-60, 60; -45, 90];
        config.gait.angles(iJoint+1,:) = 45;
        if trot
            config.gait.angles(iJoint,:) = [gait(iLeg), -gait(iLeg)];
            config.gait.feet(iJoint+1,:) = [feet(iLeg), ~feet(iLeg)];
        else
            config.gait.angles(iJoint, :) = circshift(gait2, feet2(iLeg)-1);
            config.gait.feet(iJoint+1,:) = iLeg~=feet2;
        end
        if dof(iLeg) == 3
            config.joints(:,2,iJoint+2) = y; % elbow roll axis
            config.joints(:,3,iJoint+2) = x*L{dof(iLeg)-1}(2); % link 2
            config.limits(iJoint+1, :) = [-90, 45];
            config.limits(iJoint+2, :) = [0, 135];
            config.gait.feet(iJoint+1,:) = 0;
            config.gait.angles(iJoint+1,:) = -30;
            config.gait.angles(iJoint+2,:) = 90;
            if trot
                config.gait.angles(iJoint,:) = [gait(iLeg), -gait(iLeg)];
                config.gait.feet(iJoint+2,:) = [feet(iLeg); ~feet(iLeg)];
            else
                config.gait.angles(iJoint, :) = circshift(gait2, feet2(iLeg)-1);
                config.gait.feet(iJoint+2,:) = iLeg~=feet2;
            end
        end
        config.gait.feet = logical(config.gait.feet);
        if iLeg == 1 || iLeg == 4 % mirror left legs
            config.joints(:,2:3,iJoint:end) = ...
                -config.joints(:,2:3,iJoint:end);
        end
    end
    config.parents = findParents(config.count);
    config.bodies{1} = interpolateBody(config.bodies{1}, 5);
end

function body = interpolateBody(vertices, n)
    d = 1/n:1/n:1;
    body = zeros(3, size(vertices, 2)*n);
    for i = 1:size(vertices, 2)
        p1 = vertices(:,i);
        p2 = vertices(:,1+mod(i,size(vertices,2)));
        body(:,1+(i-1)*n:i*n) = p1 + (p2-p1).*d;
    end
end

% Six-legged robot geometry with 3-DoF per leg and 2 body joints
function config = hexapod(w, h, L, horizon)
    leg = [1 1 0];
    config.count = [3, 3, 2, leg, leg, leg, leg, leg, leg];
    config.clearance = 0.0;
    body = [-w/2,w/2,w/2,-w/2; h,h,0,0; 0,0,0,0];
    config.bodies = {body, body, body};
    config.iBodies = 0:2; % indices of joint for each body segment
    config.joints = zeros(3, 3, sum(config.count));
    config.limits = zeros(sum(config.count), 2); % angle limits
    config.limits(1:2,:) = 90*[-1, 1; -1, 1]; % angle limits
    config.gait.angles = zeros(sum(config.count), 2); % gait states
    config.gait.feet = zeros(sum(config.count), 2); % gait foot contacts
    config.gait.dx = [0 0; 0.1 0.1; 0 0]; % gait centroid motion
    config.gait.dyaw = deg2rad(5); % gait max yaw correction per step
    config.gait.horizon = horizon;
    x = [1;0;0]; % right
    y = [0;1;0]; % forward
    z = [0;0;1]; % up
    gait = [-1, 1, -1, 1, -1, 1, -1, 1]*40;
    feet = [1, 0, 1, 0];
    config.joints(:,1,1:2) = [[0;h;0],[0;h;0]];
    config.joints(:,2,1:2) = [x,x];
    for iBody = 1:length(config.iBodies)
        for iLeg = 1:2
            iJoint = iLeg*3 + (iBody-1)*6;
            config.joints(1,1,iJoint) = config.bodies{iBody}(1,iLeg); % shoulder
            config.joints(2,1,iJoint) = h/2; % shoulder
            config.joints(:,2,iJoint) = z; % shoulder yaw axis
            config.joints(:,2,iJoint+1) = y; % shoulder roll axis
            config.joints(:,3,iJoint+1) = x*L; % link 1
            config.limits(iJoint:iJoint+1,:) = [-60, 60; -60, 60];
            config.gait.angles(iJoint,:) = [gait(iLeg), -gait(iLeg)];
            config.gait.feet(iJoint+1,:) = [feet(iLeg), ~feet(iLeg)];
            config.joints(:,2,iJoint+2) = y; % elbow roll axis
            config.joints(:,3,iJoint+2) = x*L; % link 2
            config.limits(iJoint+1, :) = [-45, 0];
            config.limits(iJoint+2, :) = [0, 135];
            config.gait.feet(iJoint+1,:) = [0, 0];
            config.gait.feet(iJoint+2,:) = [feet(iLeg); ~feet(iLeg)];
            config.gait.angles(iJoint+1:iJoint+2,:) = [-45, -45; 135, 135];
            config.gait.feet = logical(config.gait.feet);
            if iLeg == 1 || iLeg == 4 % mirror left legs
                config.joints(:,2:3,iJoint:end) = ...
                    -config.joints(:,2:3,iJoint:end);
            end
            if iBody == 2
                config.gait.angles(iJoint,:) = -config.gait.angles(iJoint,:);
                config.gait.feet(iJoint+2,:) = ~config.gait.feet(iJoint+2,:);
            end
        end
    end
    config.parents = findParents(config.count);
    config.bodies{1} = interpolateBody(config.bodies{1}, 5);
    config.bodies{2} = interpolateBody(config.bodies{1}, 5);
    config.bodies{3} = interpolateBody(config.bodies{1}, 5);
end