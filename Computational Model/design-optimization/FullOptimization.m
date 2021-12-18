%% Initialize
addpath('terrain-generation', 'config-generation', 'robot-kinematics', ...
    'optimization', 'visualization', 'workspace-optimization', 'motor-selection');
global FRAMES ANIMATE RECORD PLOT
FRAMES; % stores animation frames for saving video file
close all;
ANIMATE = ~~0; % flag to animate robot motion
RECORD = ~~0; % flag to save animations as video files
PLOT = ~~0; % flag to plot final robot condition and path

%% Parameters
p.simWorkspace = 1;
p.simSteps = 1;
p.workers = 4;

p.bodyLength = 0.5;
p.bodyWidth = 0.2;
p.bodyMass = 4;
p.maxDoF = 14;
p.gripperForce = 25;
p.gripperRatio = tand(20);
p.workspaceRes = 0.1;
p.workspaceCostThreshold = 0.66;

p.samples = 10;
p.steps = 8;
p.seed = 42;
p.scoreLabels = {'Success Rate', 'Normal', 'Tangential', '|F|^2', ...
    'Rat Mgn', 'Mag Mgn', 'Cost', 'Torque', 'Distance', 'Solve Time'};
% p.scoreLabels = SCORES;
p.sigma = 1;

%% Select Configurations
r = dec2bin(0:2^9-1) == '1';
fail = (r(:,8) > sum(r(:,1:3), 2)) | (r(:,5) > r(:,9));
dof = sum(r(:,1:3), 2).*(1+r(:,8)) + sum(r(:,4:6), 2)*2 + r(:,7) + r(:,9)*4 + 8;
dof(fail) = NaN;
% dof(r(:,8)) = NaN; % No second body joint
% dof(r(:,9)) = NaN; % No hexapods
configs = 0:511;
configs = configs(dof<=14);

%% Verify Controllability


%% Compute Leg Length
L0 = getMinLegLengths(configs)*p.bodyLength;
L1 = getMinLegLengths2(configs, p.bodyWidth, p.bodyLength);
if p.simWorkspace
    workspaces = getWorkspaces(configs, L1, p);
    workspaceCosts = zeros(length(configs), 3);
    for i = 1:length(configs)
        leg = workspaces(i,:);
        workspaceCosts(i,:) = [workspace2cost(leg{1}, p.workspaceRes),...
                       workspace2cost(leg{2}, p.workspaceRes),...
                       workspace2cost(leg{3}, p.workspaceRes)];
    end
    % TODO: increase length and try again if needed
end
L = L1;

%% Simulate Steps
if p.simSteps
    reuse = 0;
else
    reuse = robots;
end
[meanScores, rawScores, allScores, seeds, robots] = simulate(...
    @(var1, var2)getConfig(var1, var2, L, configs, p), ...
    @getTerrain, @getScores, @averageScores, configs, 1, p.samples, p.steps, ...
    0.25, 0, 0, p.seed, p.scoreLabels, reuse, []);
gripperCost = getDeviation(allScores(:,:,:,:,2), p.sigma)/5;
torques = getTorques(robots, seeds);

%% Select Motors
% {config, 1, sample}(step, joint)
FoS = 2;
[mname, mtorque, mmass, mcost] = loadMotors;
allMotors = cell(size(configs));
motorCosts = zeros(size(configs));
totalMasses = zeros(size(configs));
payloads = zeros(size(configs));
torqueScores = zeros(size(configs));
for i = 1:length(configs)
    torque = zeros(1, 1, size(torques, 3), size(torques{i,1,1},1), size(torques{i,1,1},2));
    for j = 1:size(torques, 3)
        torque(1,1,j,:,:) = torques{i,1,j};
    end
    torque(abs(torque)<1e-10) = NaN;
    torque = reshape(getDeviation(torque, 0)/5, [], 1)*FoS;
    
    torque2 = torque;
    ignore = 0;
    angles = robots{i,1,1}(1).config.gait.angles;
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
    err = abs(torque2-torque)./torque2;
    err(~err) = NaN;
    err = mean(err, 'omitnan');
    torque = torque2;
    torqueScores(i) = err;
    motors = optimizeMotors(torque, p.bodyMass, mname, mtorque, mmass, mcost);
    allMotors{i} = motors;
    motorCosts(i) = sum(mcost(motors));
    totalMasses(i) = p.bodyMass + sum(mmass(motors));
    payloads(i) = 1/gripperCost(i) - totalMasses(i);
end
torqueStd = sqrt(mean(torqueScores.^2));
mname(allMotors{79})
totalMasses(79)-p.bodyMass
motorCosts(79)

%% Plot Results
hold on
clear rows
[~,I] = sort(dof(dof<=p.maxDoF));
dof_filtered = dof(dof<=p.maxDoF);
lastdof = 0;
for index = 1:length(configs)
    i = I(index);
    dof_i = dof_filtered(i);
    code = dec2bin(configs(i)) == '1';
    code = [zeros(1,9-length(code)),code];
    if ~(code(4)&&code(6))
        continue
    end
    addLegend = lastdof ~= dof_i;
    lastdof = dof_i;
    codestring = num2str(code);
    codestring = codestring(codestring~= ' ');
    segs = 1+(sum(code(1:3))>0)*(1+code(8));
    c = (1:7)'/7;
    colors = [max(0,c*2-1), max(0,1-c*1.5), max(0, 1-3*abs(c-.7))];
    color = colors(dof_i-7, :);
    pry = 'PRY';
%     score = scores(i,1+(foot>1)+(foot==3));
%         if any(index == [3,12,31,42,66,108,79]) % annotate selected designs
%     score = 1/(totalMasses(i)*gripperCost(i));
%       score = mean(allScores(i,:,:,:,1),[3,4], 'omitnan');
%     score = motorCosts(i);
    score = totalMasses(i);
%     score = torqueScores(i);
    var = index;%totalMasses(i);
    var = L(i);
    if i==79%any(index == -[3,12,31,41,62,91,74]) % annotate selected designs
        plot(var, score, 'o', 'color', color, 'markersize', 10, 'linewidth',1, 'HandleVisibility','off');
    end
    if addLegend
        s = plot(var, score, '.', 'color', color, 'markersize', 20);
    else
        s = plot(var, score, '.', 'color', color, 'markersize', 20, 'HandleVisibility','off');
    end
    s.DataTipTemplate.DataTipRows(1).Label = 'i';
    s.DataTipTemplate.DataTipRows(2).Label = 'Volume';
    rows(1) = dataTipTextRow('DoF',dof_i);
    rows(2) = dataTipTextRow('Legs',4+2*code(9));
    rows(3) = dataTipTextRow('Segments',segs);
    rows(4) = dataTipTextRow('Body Joints',{(pry(code(1:3)>0))});
    rows(5) = dataTipTextRow('Knees',{code(4:6)});
    rows(6) = dataTipTextRow('Tail',code(7));
    rows(7) = dataTipTextRow('Index',i);
    rows(8) = dataTipTextRow('Code',{codestring});
    s.DataTipTemplate.DataTipRows(end+1:end+length(rows)) = rows;
end
title('Motor Optimization: 3kg');
xlabel('Index');
ylabel('Robot Mass (kg)');
legend('8-DoF', '9-DoF', '10-DoF', '11-DoF', '12-DoF', '13-DoF', '14-DoF', 'Location', 'northeast');

%% Comparison
var = 9;
clf;
hold on
clear rows
[~,I] = sort(dof(dof<=p.maxDoF));
dof_filtered = dof(dof<=p.maxDoF);
counter = 0;
lastdof = 0;
for index = 1:length(configs)
    i = I(index);
    dof_i = dof_filtered(i);
    code = dec2bin(configs(i)) == '1';
    code = [zeros(1,9-length(code)),code];

    addLegend = lastdof ~= dof_i;
    lastdof = dof_i;
    code2 = code;
    code2(var) = 1;
    codestring2 = num2str(code2);
    codestring2 = codestring2(codestring2~= ' ');
    i2 = find(bin2dec(codestring2) == configs, 1);

    if isempty(i2) || code(var)
        continue
    end
%     score2 = scores(i2,1+(foot>1)+(foot==3));
    counter = counter+1;

    codestring = num2str(code);
    codestring = codestring(codestring~= ' ');
    segs = 1+(sum(code(1:3))>0)*(1+code(8));
    c = (1:7)'/7;
    colors = [max(0,c*2-1), max(0,1-c*1.5), max(0, 1-3*abs(c-.7))];
    color = colors(dof_i-7, :);
    pry = 'PRY';
%     score = scores(i,1+(foot>1)+(foot==3));
    score = 1/(totalMasses(i)*gripperCost(i));
    score2 = 1/(totalMasses(i2)*gripperCost(i2));
    score = motorCosts(i);
    score2 = motorCosts(i2);
    score = totalMasses(i);
    score2 = totalMasses(i2);
%     score = torqueScores(i);
%     score2 = torqueScores(i2);
    varx = index;%totalMasses(i);
    
%         if any(index == [4,12,31,42,80,94]) % annotate selected designs
    if code(4) % annotate selected designs
%         plot(counter, score2-score, [color,'o'], 'markersize', 10, 'linewidth',1);
    end
    if addLegend
        s = plot(varx, score2-score, '.', 'color', color, 'markersize', 20);
    else
        s = plot(varx, score2-score, '.', 'color', color, 'markersize', 20, 'HandleVisibility','off');
    end
    s.DataTipTemplate.DataTipRows(1).Label = 'i';
    s.DataTipTemplate.DataTipRows(2).Label = 'Marginal Volume';
    rows(1) = dataTipTextRow('DoF',dof_i);
    rows(2) = dataTipTextRow('Legs',4+2*code(9));
    rows(3) = dataTipTextRow('Segments',segs);
    rows(4) = dataTipTextRow('Body Joints',{(pry(code(1:3)>0))});
    rows(5) = dataTipTextRow('Knees',{code(4:6)});
    rows(6) = dataTipTextRow('Tail',code(7));
    rows(7) = dataTipTextRow('Index',configs(i));
    rows(8) = dataTipTextRow('Code',{codestring});
    rows(9) = dataTipTextRow('Original Volume',score);
    s.DataTipTemplate.DataTipRows(end+1:end+length(rows)) = rows;
end
varnames = {'Body Pitch Joint', 'Body Roll Joint', 'Body Yaw Joint', 'Front Knees',...
    'Middle Knees', 'Back Knees', 'Tail', 'Second Body Joint', 'Middle Legs'};
title(['Effect of ',varnames{var}, ': 3kg']);
xlabel('Index');
ylabel('Total Motor Price (\$)');
ylabel('Gripper Adhesion FoS');
ylabel('Total Mass (kg)');
% ylabel('Actuator Torque (Nm)');
% legend('8-DoF', '9-DoF', '10-DoF', '11-DoF', '12-DoF', '13-DoF', '14-DoF', 'Location', 'northeast');

%% Function definitions

% User-defined robot configuration as a function of swept parameters
function config = getConfig(var1, ~, L, configs, p)
    code = dec2bin(var1) == '1';
    code = [zeros(1,9-length(code)),code];
    config = simpleWalker(code, p.bodyWidth, p.bodyLength, L(configs==var1));
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