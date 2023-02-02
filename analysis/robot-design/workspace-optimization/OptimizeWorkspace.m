global PLOT
PLOT = 1;
close all

% Set simulation parameters
simulate = ~~0; % Run the simulation instead of reusing data
parallel = ~~1; % Use parallel computation
code = 0; % Simulate a single robot design
plotIndex = 0; % Visualize a single trial
foot = 1; % Foot to analyze for single robot design
direction = 0; % Which workspace bound to plot
compare = 0; % Examine the effect of a single added DoF
res = 0.1; % Resolution of workspace (m)
w = 0.2; % Robot body width along x-axis (m)
h = 0.5; % Robot body height along y-axis (m)
l = 0.25; % Robot leg and tail length (m)
maxDoF = 14; % Maximum allowed degrees of freedom for design

% Define robot configurations
% r = dec2bin(0:2^9-1) == '1';
% fail = (r(:,8) > sum(r(:,1:3), 2)) | (r(:,5) > r(:,9));
% dof = sum(r(:,1:3), 2).*(1+r(:,8)) + sum(r(:,4:6), 2)*2 + r(:,7) + r(:,9)*4 + 8;
% dof(fail) = NaN;
% dof(r(:,9)) = NaN;
% % dof(72:end) = NaN;
% ids = 0:511;
% ids = ids(dof<=maxDoF);

ids = configs;

% Single Test
% config = simpleWalker(code, w, h, l);
% [success, state, terrain] = isPointInWorkspace(0.3,0,.2, config, foot, state);
% success
% plotTerrain(terrain);
% plotRobot(state2robot(state, config));
% return

% Simulation
if simulate
    tic();
    if code
        config = simpleWalker(code, w, h, l);
        results = getWorkspace(config, foot, res);
    elseif parallel
        allResults = cell(length(ids), 3);
        scores = zeros(length(ids), 3);
        fprintf('Running %d trials\n', length(ids));
        parfor (i = 1:length(ids),4)
            code = dec2bin(ids(i)) == '1';
            code = [zeros(1,9-length(code)),code];
            config = simpleWalker(code, w, h, l);
            codestring = num2str(code);
            codestring = codestring(codestring~= ' ');
            fprintf('Index: %d, Front Leg, %s\n', i, codestring);
            results1 = getWorkspace(config, 1, res);
            fprintf('Index: %d, Back Leg, %s\n', i, codestring);
            results2 = getWorkspace(config, 3+code(9)*2, res);
            if code(9)
                fprintf('Index: %d, Middle Leg, %s\n', i, codestring);
                results3 = getWorkspace(config, 3, res);
            else
                results3 = NaN;
            end
            scores(i,:) = [volume(results1, res),volume(results2, res),volume(results3, res)];
            allResults(i,:) = {results1, results2, results3};
        end
    else
        allResults = cell(length(ids), 3);
        scores = zeros(length(ids), 3);
        fprintf('Running %d trials\n', length(ids));
        for i = 1:length(ids)
            code = dec2bin(ids(i)) == '1';
            code = [zeros(1,9-length(code)),code];
            config = simpleWalker(code, w, h, l);
            codestring = num2str(code);
            codestring = codestring(codestring~= ' ');
            fprintf('Index: %d, Front Leg, %s\n', i, codestring);
            results1 = getWorkspace(config, 1, res);
            fprintf('Index: %d, Back Leg, %s\n', i, codestring);
            results2 = getWorkspace(config, 3+code(9)*2, res);
            if code(9)
                fprintf('Index: %d, Middle Leg, %s\n', i, codestring);
                results3 = getWorkspace(config, 3, res);
            else
                results3 = NaN;
            end
            scores(i,:) = [volume(results1, res),volume(results2, res),volume(results3, res)];
            allResults(i,:) = {results1, results2, results3};
        end
    end
    runtime = toc();
    if runtime > 3600
        fprintf('Runtime: %f hours\n', runtime/3600);
    elseif runtime > 60
        fprintf('Runtime: %f minutes\n', runtime/60);
    else
        fprintf('Runtime: %f seconds\n', runtime);
    end
end

% Compute scores
if ~code
    scores = zeros(length(ids), 3);
    for i = 1:length(ids)
        leg = allResults(i,:);
%         scores(i,:) = [volume(leg{1}, res),volume(leg{2}, res),volume(leg{3}, res)];
        scores(i,:) = [vol2cost(leg{1}, res),vol2cost(leg{2}, res),vol2cost(leg{3}, res)];
    end
end

% Plot results
if code
    V = volume(results, res);
    cost = vol2cost(results, res);
    fprintf('Volume: %.4f m^3\n', V);
    fprintf('Cost: %.4f\n', cost);
    plotWorkspace2(results, res, config, foot);
elseif plotIndex
    [~,I] = sort(dof(dof<=maxDoF));
    i = I(plotIndex);
    code = dec2bin(ids(i)) == '1';
    code = [zeros(1,9-length(code)),code];
    config = simpleWalker(code, w, h, l);
    results = allResults{i, 1+(foot>1)+(foot == 3)*code(9)};
    if width(results) == 1
        fprintf('Invalid Leg Selected');
        return
    end
    V = volume(results, res);
    cost = vol2cost(results, res);
    fprintf('Volume: %.4f m^3\n', V);
    fprintf('Cost: %.4f\n', cost);
    plotWorkspace2(results, res, config, foot);
elseif compare
    plotComparison(compare, scores, foot, dof, maxDoF, ids);
else
    hold on
    clear rows
    [~,I] = sort(dof(dof<=maxDoF));
    dof_filtered = dof(dof<=maxDoF);
    for index = 1:size(scores,1)
        i = I(index);
        dof_i = dof_filtered(i);
        code = dec2bin(ids(i)) == '1';
        code = [zeros(1,9-length(code)),code];
        if code(9)
            continue
        end
        codestring = num2str(code);
        codestring = codestring(codestring~= ' ');
        segs = 1+(sum(code(1:3))>0)*(1+code(8));
        colors = ['r','b','m','k','r','b','m'];
        color = colors(dof_i-7);
        pry = 'PRY';
        score = scores(i,1+(foot>1)+(foot==3));
%         if any(index == [3,12,31,42,66,108,79]) % annotate selected designs
        if any(index == [3,12,31,41,62,91,74]) % annotate selected designs
            plot(index, score, [color,'o'], 'markersize', 10, 'linewidth',1);
        end
        s = plot(index, score, [color,'.'], 'markersize', 20);
        s.DataTipTemplate.DataTipRows(1).Label = 'i';
        s.DataTipTemplate.DataTipRows(2).Label = 'Volume';
        rows(1) = dataTipTextRow('DoF',dof_i);
        rows(2) = dataTipTextRow('Legs',4+2*code(9));
        rows(3) = dataTipTextRow('Segments',segs);
        rows(4) = dataTipTextRow('Body Joints',{(pry(code(1:3)>0))});
        rows(5) = dataTipTextRow('Knees',{code(4:6)});
        rows(6) = dataTipTextRow('Tail',code(7));
        rows(7) = dataTipTextRow('Index',ids(i));
        rows(8) = dataTipTextRow('Code',{codestring});
        s.DataTipTemplate.DataTipRows(end+1:end+length(rows)) = rows;
    end
    if foot <= 2
        title('Workspace: Front Leg');
    elseif foot == 3
        title('Workspace: Middle Leg');
    else
        title('Workspace: Back Leg');
    end
    xlabel('Index');
    ylabel('Volume (m$^3$)');
    ylabel('Gripper Adhesion Cost');
%     ylim([0.605,0.647])
%     ylim([0.585,0.647])
end


% Compute workspace for a single limb
function results = getWorkspace(config, foot, res)
    neighbors = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1]*res;
    dx = [0,0,0];
    [success, state, terrain] = isPointInWorkspace(0, 0, 0, config, foot);
    if ~success
        fprintf('Initial point failed');
        results = table(dx, [NaN,NaN], terrain);
        return
    end
    r = state2robot(state, config);
    feet = r.vertices(:, sum(config.gait.feet, 2) > 0);
    results = table(dx, success, state', terrain);
    edges = table(dx + neighbors, repmat(state', size(neighbors,1), 1), vecnorm(dx+neighbors,2,2));

    while size(edges, 1) > 0
%         fprintf('Nodes: %d, Edges: %d\n', size(results,1), size(edges,1));
        [~, imin] = min(edges{:,3});
        dx = edges{imin,1};
        state = edges{imin,2};
        edges(imin,:) = [];
        
        skip = 0;
%         for dist = 0:floor(norm(dx)/res)-1
%             p = round(dist*dx/norm(dx))*res;
%             ip = all(abs(p - results{:,1}) < res/2, 2);
%             if sum(ip) == 0 || ~results{ip,2}
%                 skip = 1;
%                 disp(dx);
%                 break
%             end
%         end
        if dx(2) < 0 || dx(1) + feet(1,foot) > 0
            skip = 1;
        end
        if skip
            results(end+1,:) = {dx, 0, state, terrain};
            continue
        end
        
        [success, state, terrain] = isPointInWorkspace(dx(1), dx(2), dx(3), config, foot, state');
        results(end+1,:) = {dx, success, state', terrain};
        if success
            for i = 1:length(neighbors)
                if ~any(all(abs(dx + neighbors(i,:) - [results{:,1}; edges{:,1}]) < res/2, 2))
                    edges(end+1, :) = {dx + neighbors(i,:), state', norm(dx+neighbors(i,:))};
                    disp = edges(end,:);
                end
            end
        end
    end
end

% Convert list of points into meshgrid
function [X, Y, Zmin, Zmax] = process(results, res)
    dx = results{:,1};
    z = results{:,2};

    xrange = min(dx(:,1)):res:max(dx(:,1));
    yrange = min(dx(:,2)):res:max(dx(:,2));
    [X,Y] = meshgrid(xrange, yrange);
    Zmin = X*NaN;
    Zmax = X*NaN;
    for i = 1:size(dx,1)
        if ~(z(i,1) < z(i,2))
            continue
        end
        i1 = round((dx(i,1)-xrange(1))/res + 1);
        i2 = round((dx(i,2)-yrange(1))/res + 1);
        Zmin(i2, i1) = z(i,1);
        Zmax(i2, i1) = z(i,2);
    end
end

% Display the workspace of a single limb
function plotWorkspace(results, res, config, foot, direction)
    [X, Y, Zmin, Zmax] = process(results, res);
    [~, state, ~] = getVerticalWorkspace(0, 0, config, foot, 1);
    r1 = state2robot(state, config);
    [~, state, ~] = getVerticalWorkspace(0, 0, config, foot, -1);
    r2 = state2robot(state, config);
    feet = r1.vertices(:, sum(config.gait.feet, 2) > 0);

    if direction >= 0
        surf(X+feet(1,foot), -Zmax, Y+feet(2,foot), 'FaceAlpha',.5,'EdgeAlpha',.7);
        plotRobot(r1);
    end
    if direction <= 0
        surf(X+feet(1,foot), -Zmin, Y+feet(2,foot), 'FaceAlpha',.5,'EdgeAlpha',.7);
        plotRobot(r2);
    end
    axis equal;
end

% Display the workspace of a single limb
function plotWorkspace2(results, res, config, foot)
    success = results{:,2};
    states = results{success,3};
    terrains = results{success,4};
    r = state2robot(states(1,:)', config);
    feet = r.vertices(:, sum(config.gait.feet, 2) > 0);
    dx = results{:,1};
    X = dx(success,1);
    Y = dx(success,2);
    Z = dx(success,3);
    hold on;
    for i = 1:length(X)
        plotCube(X(i)+feet(1,foot), -Z(i), Y(i)+feet(2,foot), res, 'red')
    end
    
    plotRobot(r);
    
    [~,imax] = max(Z);
    r = state2robot(states(imax,:)', config);
%     plotRobot(r);
    
    [~,imin] = max(X-2*Y);
    r = state2robot(states(imin,:)', config);
    terrain = terrains(imin);
    grid = sphereTerrain([-.5,.5], [-1,.5], 0.01, terrain.center, terrain.radius, terrain.sign, [0;0;0]);
%     plotTerrain(grid);
%     plotRobot(r);
    view([0,0])
    axis equal;
end

function V = volume(results, res)
    if width(results) == 1
        V = NaN;
        return
    end
%     V = sum(results{:,2}(:,2) - results{:,2}(:,1), 'omitnan')*res^2;
    V = sum(results{:,2})*res^3;
end

function [cost, costs] = vol2cost(results, res)
    if width(results) == 1
        cost = NaN;
        costs = NaN;
        return
    end
    z0 = min(results{:,1}(:,3));
    zmax = max(results{:,1}(:,3));
    z = z0:res:zmax;
    costs = zeros(length(z), 1);
    for i = 1:length(z)
        layer = abs(results{:,1}(:,3) - z(i)) < res/2;
        A = sum(results{layer,2})*res.^2;
        costs(i) = area2cost(A);
    end
%     cost = sum(area2cost(0) - costs);
%     cost = (area2cost(0) - cost/8);
    cost = min(costs);
end

function plotCube(x, y, z, side, color)
    vertices = ones(3,2,2,2);
    vertices(1,1,:,:) = -1;
    vertices(2,:,1,:) = -1;
    vertices(3,:,:,1) = -1;
    vertices = vertices*side/2 + [x;y;z];
    sides = {vertices(:,1,:,:), vertices(:,2,:,:), vertices(:,:,1,:), vertices(:,:,2,:), vertices(:,:,:,1), vertices(:,:,:,2)};
    for i = 1:6
        s = reshape(sides{i}, 3, []);
        s = [s(:,1),s(:,2),s(:,4),s(:,3)];
        p1 = patch(s(1,:), s(2,:), s(3,:), 1, 'FaceAlpha',.1);
        p1.FaceColor = color;
        p1.EdgeColor = 'black';
    end
end

function plotComparison(var, scores, foot, dof, maxDoF, ids)
    hold on
    clear rows
    [~,I] = sort(dof(dof<=maxDoF));
    dof_filtered = dof(dof<=maxDoF);
    counter = 0;
    for index = 1:size(scores,1)
        i = I(index);
        dof_i = dof_filtered(i);
        code = dec2bin(ids(i)) == '1';
        code = [zeros(1,9-length(code)),code];
        
        code2 = code;
        code2(var) = 1;
        codestring2 = num2str(code2);
        codestring2 = codestring2(codestring2~= ' ');
        i2 = find(bin2dec(codestring2) == ids, 1);
        
        if isempty(i2) || code(var)
            continue
        end
        score2 = scores(i2,1+(foot>1)+(foot==3));
        counter = counter+1;
        
        codestring = num2str(code);
        codestring = codestring(codestring~= ' ');
        segs = 1+(sum(code(1:3))>0)*(1+code(8));
        colors = ['r','b','m','k','r','b','m'];
        color = colors(dof_i-7);
        pry = 'PRY';
        score = scores(i,1+(foot>1)+(foot==3));
%         if any(index == [4,12,31,42,80,94]) % annotate selected designs
        if code(8) % annotate selected designs
            plot(counter, score2-score, [color,'o'], 'markersize', 10, 'linewidth',1);
        end
        s = plot(counter, score2-score, [color,'.'], 'markersize', 20);
        s.DataTipTemplate.DataTipRows(1).Label = 'i';
        s.DataTipTemplate.DataTipRows(2).Label = 'Marginal Volume';
        rows(1) = dataTipTextRow('DoF',dof_i);
        rows(2) = dataTipTextRow('Legs',4+2*code(9));
        rows(3) = dataTipTextRow('Segments',segs);
        rows(4) = dataTipTextRow('Body Joints',{(pry(code(1:3)>0))});
        rows(5) = dataTipTextRow('Knees',{code(4:6)});
        rows(6) = dataTipTextRow('Tail',code(7));
        rows(7) = dataTipTextRow('Index',ids(i));
        rows(8) = dataTipTextRow('Code',{codestring});
        rows(9) = dataTipTextRow('Original Volume',score);
        s.DataTipTemplate.DataTipRows(end+1:end+length(rows)) = rows;
    end
    varnames = {'Body Pitch Joint', 'Body Roll Joint', 'Body Yaw Joint', 'Front Knees',...
        'Middle Knees', 'Back Knees', 'Tail', 'Second Body Joint', 'Middle Legs'};
    if foot <= 2
        title(['Effect of ',varnames{var},'']);
    elseif foot == 3
        title(['Marginal Improvement of ',varnames{var},': Middle Leg']);
    else
        title(['Marginal Improvement of ',varnames{var},': Back Leg']);
    end
    xlabel('Index');
    ylabel('Adhesion Cost');
    ylim([-0.005,0.015]);
end