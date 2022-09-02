figure(1);
rng(42);

x0 = [1; 1]; % initial state
xg = [9; 9]; % goal state
xmin = [0; 0]; % state space lower bound
xmax = [10; 10]; % state space upper bound
iters = 5000; % iterations
m = 100; % samples per batch

robot.L0 = 0.5; % default leg length
robot.dL = 0.4; % maximum change in leg length
robot.theta0 = [45, 135, 225, 315]; % default leg angles
robot.dtheta = 30; % maximum change in leg angles

n = size(x0, 1); % state space dimension
vol = prod(xmax-xmin); % volume of free space
X = [x0, xg]; % indexed array of states nxN
g = 2:length(X); % goal indices
Xun = g; % unconnected nodes 1xN
V = 1; % RGG vertices 1xN
Vp = [0; 0]; % RGG vertex parent indices and edge costs 2xN
E = zeros(3,0); % RGG edges + costs 3xN
q = length(V) + length(Xun); % number of samples
infl = inf; % inflation factor
trunc = inf; % truncation factor
Vcl = []; % closed (already explored) vertices 1xN
Vin = []; % inconsistent vertices 1xN
Q = expand(1, V, Vp, E, X, Xun, inf, g); % edge queue 2xN
iterCost = zeros(1, iters) + nan;

obs = rand(xmax(1)-xmin(1)+1, xmax(2)-xmin(2)+1) > 1/5;
obs(x0(1)+1, x0(2)+1) = 1;
for i = 1:size(xg, 2)
    obs(xg(1,i)+1, xg(2,i)+1) = 1;
end
    
searchFinished = true;
lastCost = inf;
for iter = 1:iters
    if searchFinished
        if lastCost == min(iterCost)
%             [V, E, Xun] = prune(V, Vp, E, X, Xun, g);
            X = [X, sample(m, min(iterCost), x0, xg, xmin, xmax, robot, obs)];
            Xun = [Xun, length(X)-m+1:length(X)];
            q = length(V) + length(Xun);
            Q = [Q, expand(1, V, Vp, E, X, Xun, radius(q, vol, n), g)];
        else
            Q = [Q, expand(Vin, V, Vp, E, X, Xun, radius(q, vol, n), g)];
            lastCost = min(iterCost);
        end
        [infl, trunc] = updateParameters(q, min(iterCost));
        Vcl = [];
        Vin = [];
        searchFinished = false;
    else
        Xp = X(:, Q(1,:));
        Xc = X(:, Q(2,:));
        [~, I] = min(treeCost(1, Q(1,:), V, Vp, E) + edgeHeuristic(Xp, Xc) + ...
            infl*goalHeuristic(Xc, xg));
        edge = Q(:, I);
        Q(:,I) = [];
        if any(all(E(1:2,:) == edge, 1))
            if any(Vcl == edge(2))
                Vin = [Vin, edge(2)];
            else
                Q = [Q, expand(edge(2), V, Vp, E, X, Xun, ...
                    radius(q, vol, n), g)];
                Vcl = [Vcl, edge(2)];
            end
        else
            xp = X(:, edge(1));
            xc = X(:, edge(2));
            gtp = treeCost(1, edge(1), V, Vp, E);
            chat = edgeHeuristic(xp, xc);
            hhat = goalHeuristic(xc, xg);
            best = min(treeCost(1, g, V, Vp, E));
            if trunc*(gtp + chat + hhat) <= best
                gtc = treeCost(1, edge(2), V, Vp, E);
                if gtp + chat < gtc                    
                    c = edgeCost(xp, xc, obs);
                    if gtp + c + hhat < best && gtp + c < gtc
                        if any(V == edge(2))
                            E(:, E(2,:) == edge(2)) = [];
                            ip = find(V == edge(1), 1);
                            ic = find(V == edge(2), 1);
                            Vp(:, ic) = [ip; c];
                        else
                            Xun(Xun == edge(2)) = [];
                            V = [V, edge(2)];
                            ip = find(V == edge(1), 1);
                            Vp = [Vp, [ip; c]];
                        end
                        E = [E, [edge; c]];
                        if any(Vcl == edge(2))
                            Vin = [Vin, edge(2)];
                        else
                            Q = [Q, expand(edge(2), V, Vp, E, X, Xun, ...
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
    iterCost(iter) = min(treeCost(1, g, V, Vp, E));
    if mod(iter, 500) == 0
        fprintf('%d: error = %.3f\n', iter, iterCost(iter));
        plotPath(E, V, Vp, X, Q, x0, xg, xmin, xmax, obs, robot);
        title(['Iteration: ', num2str(iter)]);
        drawnow();
    end
end

path = getTree(1, 2, V, Vp, E);
totalCost = 0;
C = zeros(2,0);
for edge = 1:length(path)-1
    e1 = X(:,V(path(edge)));
    e2 = X(:,V(path(edge+1)));
    fprintf('Computing edge %d/%d\n', edge, length(path)-1);
    [footholds, cost] = computeEdgeFootholds(e1, e2, 90, 90, xmin, xmax, robot, obs);
    totalCost = totalCost + cost;
    figure(1);
    plotEdge(footholds, robot);
    [centroid, heading, ~] = getRobotPose(footholds, robot);
    C = [C, centroid];
end
plot(C(1,:), C(2,:), 'c-', 'linewidth', 5);
fprintf('Total error: %d\n', totalCost);

function plotPath(E, V, Vp, X, Q, x0, xg, xmin, xmax, obs, robot)
    clf;
    colormap gray
    hold on;
    axis equal;
    imagesc(xmin(2):xmax(2), xmin(1):xmax(1), obs');
%     for i = 1:size(Q, 2)
%         plot(X(1,Q(1:2,i)), X(2,Q(1:2,i)), 'r');
%     end
    for i = 1:size(E, 2)
        plot(X(1,E(1:2,i)), X(2,E(1:2,i)), 'm');
    end
    plot(X(1,V), X(2,V), 'm.');
    path = getTree(1, 2, V, Vp, E);
%     plot(X(1,V(path)), X(2,V(path)), 'c-', 'linewidth', 5);
    plot(x0(1,:), x0(2,:), 'g.', 'markersize', 25);
    plotRobot(createState(x0, 90, robot), 'g', robot);
    plot(xg(1,:), xg(2,:), 'r.', 'markersize', 25);
    plotRobot(createState(xg, 90, robot), 'r', robot);
    xlim([xmin(1), xmax(1)]);
    ylim([xmin(2), xmax(2)]);
end

function plotEdge(footholds, robot)
    for i = 1:size(footholds, 2)
        plotRobot(footholds(:,i,:), 'b', robot);
    end
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

function F = createState(centroid, heading, robot)
    theta = heading + robot.theta0;
    F = centroid + robot.L0*[cosd(theta); sind(theta)];
    F = reshape(F, [], 1, 4);
end

function Eout = expand(Xi, V, Vp, E, X, Xun, r, g)
    Eout = [];
    x0 = X(:,1);
    xg = X(:,g);
    best = min(treeCost(1, g, V, Vp, E));
    for p = Xi
        Eout = [Eout, E(1:2, E(1,:) == p)];
        Xall = [Xun, V];
        xp = X(:, p);
        d = xp - X(:,Xall);
        ghatp = startHeuristic(x0, xp);
        c = Xall(sum(d.^2, 1) <= r^2);
        gtc = treeCost(1, c, V, Vp, E);
        xc = X(:, c);
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

function [V, E, Xun] = prune(V, Vp, E, X, Xun, g)
    best = min(treeCost(1, g, V, Vp, E));
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
        infl = 1e6;
    end
end

function x = sample(m, cost, x0, xg, xmin, xmax, robot, obs)
    x = rand(length(xmin), m) .* (xmax - xmin) + xmin;
    for i = 1:100
        I = startHeuristic(x0, x) + goalHeuristic(x, xg) > cost;
        f1 = x + robot.L0*[cosd(robot.theta0(1)); sind(robot.theta0(1))];
        f2 = x + robot.L0*[cosd(robot.theta0(2)); sind(robot.theta0(2))];
        f3 = x + robot.L0*[cosd(robot.theta0(3)); sind(robot.theta0(3))];
        f4 = x + robot.L0*[cosd(robot.theta0(4)); sind(robot.theta0(4))];
        
        I = I | isObstacle(x, obs) | isObstacle(f1, obs) |...
            isObstacle(f2, obs) | isObstacle(f3, obs) | isObstacle(f4, obs);
        n = sum(I);
        if n == 0
            return
        end
        x(:, I) = rand(length(xmin), n) .* (xmax - xmin) + xmin;
    end
end

function invalid = isObstacle(X, obs)
    i = round(X(1,:)) + 1;
    j = round(X(2,:)) + 1;
    invalid = ~obs(sub2ind(size(obs), i, j));
end

function gt = treeCost(x0, Xi, V, Vp, E)
    gt = zeros(size(Xi)) + inf; % g indexed
    for i = 1:length(Xi)
        I = find(V == Xi(i), 1);
        if ~isempty(I)
            gt(i) = Vp(2,I);
            ip = Vp(1,I);
            while ip > 0
                gt(i) = gt(i) + Vp(2,ip);
                ip = Vp(1,ip);
            end
        end
    end
end

function path = getTree(x0, Xg, V, Vp, E)
    path = find(V == Xg, 1);
    if ~isempty(path)
        ip = Vp(1,path(1));
        while ip > 0
            path = [ip, path];
            ip = Vp(1,path(1));
        end
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
    chat = vecnorm(xp-xc);
end

function c = edgeCost(xp, xc, obs)
    c = vecnorm(xp-xc);
    if c > 1
        c = inf;
        return
    end
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