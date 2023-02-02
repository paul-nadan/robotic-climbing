figure(1);
plotPath(E, V, X, C, Q, F, g, xmin, xmax, obs, robot);
return

% Simulation parameters
iters = 100000; % iterations
mf = 30; % footholds per batch
mx = 500; % sets per batch

xmin = [0; 0]; % state space lower bound
xmax = [4; 4]; % state space upper bound
x0 = [2; 1]; % initial state
xg = [2; 2]; % goal state

robot.L0 = 1; % default leg length
robot.dL = 0.8; % maximum change in leg length
robot.theta0 = [45, 135, 225, 315]; % default leg angles
robot.dtheta = 30; % maximum change in leg angles

% Variable definition
n = size(x0, 1); % state space dimension
vol = prod(xmax-xmin); % volume of free space
F = createState(x0, 90, robot); % array of footholds nxNx4
for i = 1:size(xg, 2)
    F = [F, createState(xg(:,i), 90, robot)];
end
X = repmat(1:size(xg, 2)+1, 4, 1); % foothold sets 4xN
C = [x0, xg]; % centroid positions nxN
g = 2:size(X, 2); % goal indices
Xun = g; % unconnected nodes 1xN
V = 1; % RGG vertices 1xN
E = zeros(3,0); % RGG edges + costs 3xN
q = length(V) + length(Xun); % number of samples
infl = inf; % inflation factor
trunc = inf; % truncation factor
Vcl = []; % closed (already explored) vertices 1xN
Vin = []; % inconsistent vertices 1xN
Q = expand(1, V, E, X, Xun, C, inf, g); % edge queue 2xN
iterCost = zeros(1, iters) + nan; % cost of best path at each iteration

% Obstacle generation
obs = rand(xmax(1)-xmin(1)+1, xmax(2)-xmin(2)+1) > 0/5;
obs(x0(1)+1, x0(2)+1) = 1;
for i = 1:size(xg, 2)
    obs(xg(1,i)+1, xg(2,i)+1) = 1;
end

% Algorithm

F = [F, sampleFootholds(mf, xmin, xmax, obs)];
% [x, c] = sampleSets(mx, F, robot);
% X = [X, x];
% C = [C, c];

% plotPath(E, V, X, C, Q, F, g, xmin, xmax, obs, robot);
searchFinished = true;
lastCost = inf;
for iter = 1:iters
    if searchFinished
        if lastCost == min(iterCost)
%             [V, E, Xun] = prune(V, E, X, Xun, g);
%             m = max(0, min(mf, size(X,2)*mf - size(F,2)*mx + mf));
%             F = [F, sampleFootholds(m, xmin, xmax, obs)];
            [x, c] = sampleSets(mx, F, robot);
            X = [X, x];
            C = [C, c];
            [~, u] = unique(X', 'rows', 'stable');
            X = X(:,u);
            C = C(:,u);
            Xun = [Xun, length(X)-size(x,2)+1:length(X)];
            q = length(V) + length(Xun);
            Q = [Q, expand(1, V, E, X, Xun, C, radius(q, vol, n), g)];
        else
            Q = [Q, expand(Vin, V, E, X, Xun, C, radius(q, vol, n), g)];
            lastCost = min(iterCost);
        end
        [infl, trunc] = updateParameters(q, min(iterCost));
        Vcl = [];
        Vin = [];
        searchFinished = false;
    else
        Xp = C(:, Q(1,:));
        Xc = C(:, Q(2,:));
        [~, I] = min(treeCost(1, Q(1,:), V, E) + edgeHeuristic(Xp, Xc) + ...
            infl*goalHeuristic(Xc, xg));
        edge = Q(:, I);
        Q(:,I) = [];
        if any(all(E(1:2,:) == edge, 1))
            if any(Vcl == edge(2))
                Vin = [Vin, edge(2)];
            else
                Q = [Q, expand(edge(2), V, E, X, Xun, C, ...
                    radius(q, vol, n), g)];
                Vcl = [Vcl, edge(2)];
            end
        else
            xp = C(:, edge(1));
            xc = C(:, edge(2));
            gtp = treeCost(1, edge(1), V, E);
            chat = edgeHeuristic(xp, xc);
            hhat = goalHeuristic(xc, xg);
            best = min(treeCost(1, g, V, E));
            if trunc*(gtp + chat + hhat) <= best
                gtc = treeCost(1, edge(2), V, E);
                if gtp + chat < gtc                    
                    c = edgeCost(xp, xc, obs);
                    if gtp + c + hhat < best && gtp + c < gtc
                        if any(V == edge(2))
                            E(:, E(2,:) == edge(2)) = [];
                        else
                            Xun(Xun == edge(2)) = [];
                            V = [V, edge(2)];
                        end
                        E = [E, [edge; c]];
                        if any(Vcl == edge(2))
                            Vin = [Vin, edge(2)];
                        else
                            Q = [Q, expand(edge(2), V, E, X, Xun, C, ...
                                radius(q, vol, n), g)];
                            Vcl = [Vcl, edge(2)];
                        end
                    end
                end         
            else
                searchFinished = true;
            end
        end
    end
    if isempty(Q)
        searchFinished = true;
    end
    iterCost(iter) = min(treeCost(1, g, V, E));
    if mod(iter, 500) == 0
        fprintf('%d: error = %.3f, f = %d, x = %d\n', iter, ...
            iterCost(iter), size(F,2), size(X,2));
        plotPath(E, V, X, C, Q, F, g, xmin, xmax, obs, robot)
        title(['Iteration: ', num2str(iter)]);
        drawnow();
    end
end

function plotPath(E, V, X, C, Q, F, g, xmin, xmax, obs, robot)
    clf;
    colormap gray
    hold on;
    axis equal;
    imagesc(xmin(2):xmax(2), xmin(1):xmax(1), obs');
%     for i = 1:size(Q, 2)
%         plot(X(1,Q(1:2,i)), X(2,Q(1:2,i)), 'r');
%     end
    % TODO: plot robot states and footholds, not just centroids
%     for i = 1:size(E, 2)
%         plot(C(1,E(1:2,i)), C(2,E(1:2,i)), 'k');
%     end
%     for i = 1:length(V)
%         plot(C(1,V(i)), C(2,V(i)), 'k.');
%     end
    for i = 1:4
        plot(F(1,:,i), F(2,:,i), '.');
    end
    plotRobot(F(:,1,:), 'g', robot)
    plotRobot(F(:,g,:), 'r', robot)
    
    path = getTree(1, 2, V, E);
    
    Fx = indexFeet(X(:,V(:,path)), F);
%     plotRobot(Fx(:,g(end)+1:end,:), 'b', robot)
    if size(Fx,2) > 0
       plotRobot(Fx(:,2:end-1,:), 'b', robot)
    end
    plot(C(1,V(path)), C(2,V(path)), 'c-', 'linewidth', 5);
    xlim([xmin(1), xmax(1)]);
    ylim([xmin(2), xmax(2)]);
end

function plotRobot(feet, color, robot)
    for j = 1:size(feet,2)
        [centroid, heading, ~] = getRobotPose(feet(:,j,:), robot);
        R = [cosd(heading), -sind(heading); sind(heading), cosd(heading)];
        corners = centroid + ...
            (robot.L0 - robot.dL)*R*[1 1; -1 1; -1 -1; 1 -1; 1 1]';
        plot(corners(1,:), corners(2,:), color);
        for i = 1:4
            plot([corners(1,i), feet(1,j,i)], ...
                [corners(2,i), feet(2,j,i)], color);
        end
        marker = mean(corners(:, 4:5), 2);
        plot(marker(1), marker(2), [color, '.'], 'markersize', 20);
    end
end

function [centroid, heading, invalid] = getRobotPose(feet, robot)
    centroid = mean(feet, 3);
    df = feet - centroid;
    a = mod(atan2d(df(2,:,:), df(1,:,:)) - ...
        reshape(robot.theta0, 1,1,[]) + 180, 360) - 180;
    heading = mean(a, 3);
    invalid = any(abs(vecnorm(df) - robot.L0) > robot.dL, 3) | ...
        any(abs(heading - a) > robot.dtheta, 3);
end

function Eout = expand(Xi, V, E, X, Xun, C, r, g)
    r = inf;
    Eout = [];
    x0 = C(:,1);
    xg = C(:,g);
    best = min(treeCost(1, g, V, E));
    for p = Xi
        valid = sum(X(:,p) == X(:,E(2,:))) == 3;
        Eout = [Eout, E(1:2, E(1,:) == p & valid)];
        Xall = [Xun, V];
        xp = C(:, p);
        valid = sum(X(:,p) == X(:,Xall)) == 3;
        Xall = Xall(valid);
        d = xp - C(:,Xall);
        ghatp = startHeuristic(x0, xp);
        c = Xall(sum(d.^2, 1) <= r^2);
        gtc = treeCost(1, c, V, E);
        xc = C(:, c);
        chat = edgeHeuristic(xp, xc);
        hhat = goalHeuristic(xc, xg);
        for i = 1:length(c)
            if ghatp + chat(i) + hhat(i) <= best
                if ghatp + chat(i) <= gtc(i)
                    Eout = [Eout, [p; c(i)]];
                end
            end
        end
    end
end

function [V, E, Xun] = prune(V, E, X, Xun, g)
    best = min(treeCost(1, g, V, E));
    fhatX = startHeuristic(X(:,1), X(:,Xun)) + ...
        goalHeuristic(X(:,Xun), X(:,g));
    fhatV = startHeuristic(X(:,1), X(:,V)) + ...
        goalHeuristic(X(:,V), X(:,g));
    fhatE1 = startHeuristic(X(:,1), X(:,E(1,:))) + ...
        goalHeuristic(X(:,E(1,:)), X(:,g));
    fhatE2 = startHeuristic(X(:,1), X(:,E(2,:))) + ...
        goalHeuristic(X(:,E(2,:)), X(:,g));
    Xun(fhatX >= best) = [];
    V(fhatV >= best) = [];
    E(:,fhatE1 >= best | fhatE2 >= best) = [];
    % TODO: move unconnected nodes from V to Xun
end

function r = radius(q, vol, n)
    eta = 1.1; % RGG constant
    ball = pi^(n/2)/gamma(n/2+1); % volume of unit ball
    r = eta*(2*(1+1/n)*(vol/ball)*(log(q)/q))^(1/n);
end

function [infl, trunc] = updateParameters(q, cost)
    infl = 1 + 10/q;
    trunc = 1 + 5/q;
    if cost == inf
        trunc = 1e6;
    end
end

function F = createState(centroid, heading, robot)
    theta = heading + robot.theta0;
    F = centroid + robot.L0*[cosd(theta); sind(theta)];
    F = reshape(F, [], 1, 4);
end

function F = sampleFootholds(m, xmin, xmax, obs)
    F = rand(length(xmin), m, 4) .* (xmax - xmin) + xmin;
    for k = 1:4
        i = round(F(1,:,k)) + 1;
        j = round(F(2,:,k)) + 1;
        F(:, ~obs(sub2ind(size(obs), i, j)), k) = inf;
    end
end

function [X, C] = sampleSets(m, F, robot)
    X = zeros(4,m);
    n = size(F, 2);
    r = (robot.L0+robot.dL)*[sqrt(2), 2, sqrt(2)];
    X(1,:) = randsample(n, m, true);
    for i = 1:m
        p = F(:, X(1,i), 1);
        for j = 2:4
            f = p - F(:,:,j);
            f = find(sum(f.^2) < r(j-1)^2);
            if isempty(f)
                X(1,i) = -1;
                break;
            elseif length(f) == 1
                X(j,i) = f;
            else
                X(j,i) = randsample(f, 1);
            end
        end
    end
    X(:, X(1,:) == -1) = [];
    [C, ~, invalid] = getRobotPose(indexFeet(X, F), robot);
    X(:,invalid) = [];
    C(:,invalid) = [];
    % TODO: remove duplicates
end

function Fx = indexFeet(X, F)
    Fx = zeros(size(F,1), size(X,2), size(F,3));
    for i = 1:4
        Fx(:,:,i) = F(:,X(i,:),i);
    end
end

function gt = treeCost(x0, Xi, V, E)
    cost = zeros(size(V)) + inf; % V indexed
    cost(V == x0) = 0;
    unexplored = ones(size(cost)); % V indexed
    gt = zeros(size(Xi)) + inf; % g indexed
    for i = 1:length(V)
        [c,I] = min(cost.*unexplored); % V indexed
        e = E(1,:) == V(I); % E indexed
        for v = find(e)
            cost(V==E(2,v)) = min(cost(V==E(2,v)), c + E(3,v));
        end
        unexplored(I) = inf;
    end
    for i = 1:length(Xi)
        [~,I] = find(V == Xi(i));
        if ~isempty(I)
            gt(i) = cost(I);
        end
    end
end

function path = getTree(x0, xg, V, E)
    cost = zeros(size(V)) + inf; % V indexed
    cost(V == x0) = 0;
    unexplored = ones(size(cost)); % V indexed
    for i = 1:length(V)
        [c,I] = min(cost.*unexplored); % V indexed
        e = E(1,:) == V(I); % E indexed
        for v = find(e)
            cost(V==E(2,v)) = min(cost(V==E(2,v)), c + E(3,v));
        end
        unexplored(I) = inf;
    end
    [~,g] = find(V == xg);
    if isempty(g)
        path = [];
        return
    end
    c = cost(I);
    path = I;
    while c > 0
        e = E(2,:) == V(path(1));
        e = E(1,e);
        v = zeros(size(e));
        for i = 1:length(v)
            v(i) = find(V==e(i));
        end
        [c,I] = min(cost(v));
        path = [v(I), path];
    end
end

function ghat = startHeuristic(x0, x)
    ghat = vecnorm(x-x0);
end

function hhat = goalHeuristic(x, xg)
    hhat = zeros(1, size(x,2))+inf;
    for xgi = xg
        hhat = min(hhat, vecnorm(xgi-x));
    end
end

function chat = edgeHeuristic(xp, xc)
    chat = vecnorm(xp-xc)+1;
end

function c = edgeCost(xp, xc, obs)
    c = vecnorm(xp-xc)+1;
    dstep = 0.01;
    S = 0:dstep:c;
    for s = S
        x = xp + (xc-xp)*s/c;
        i = round(x(1)) + 1;
        j = round(x(2)) + 1;
        if ~obs(i, j)
            c = inf;
            return;
        end
    end
end