function [feet, cost] = computeEdgeFootholds(x0, xg, h0, hg, Xmin, Xmax, robot, obs)
    % Simulation parameters
    iters = 2000; % iterations
    m = 5; % footholds per batch
    w = robot.L0 + robot.dL; % maximum distance from edge to sample
    l = norm(xg-x0);

    xmin = [-w; -w]; % state space lower bound
    xmax = [w; w+l]; % state space upper bound

    % Variable definition
    n = size(x0, 1); % state space dimension
    vol = prod(xmax-xmin); % volume of free space
    F = createState(x0, h0, robot); % array of footholds nxNx4
    for i = 1:size(xg, 2)
        F = [F, createState(xg(:,i), hg(i), robot)];
    end
    X = repmat(1:size(xg, 2)+1, 4, 1); % foothold sets 4xN
    C = [x0, xg]; % centroid positions nxN
    g = 2:size(X, 2); % goal indices
    Xun = g; % unconnected nodes 1xN
    V = 1; % RGG vertices 1xN
    Vp = [0; 0]; % RGG vertex parent indices and edge costs 2xN
    E = zeros(3,0); % RGG edges + costs 3xN
    q = length(V) + length(Xun); % number of samples
    infl = inf; % inflation factor
    trunc = inf; % truncation factor
    Vcl = []; % closed (already explored) vertices 1xN
    Vin = []; % inconsistent vertices 1xN
    Q = expand(1, V, Vp, E, X, Xun, C, inf, g); % edge queue 2xN
    iterCost = zeros(1, iters) + nan; % cost of best path at each iteration
    minCost = inf;

    % Algorithm
    F = [F, sampleFootholds(m, xmin, xmax, x0, xg, obs)];
    [x, c] = generateSets(F, [], robot);
    X = [X, x];
    C = [C, c];

    figure(2);
    plotPath(E, V, Vp, X, C, Q, F, g, xmin, xmax, obs, robot);
    searchFinished = true;
    lastCost = inf;
    for iter = 1:iters
        if searchFinished
            if lastCost == minCost
    %             [V, E, Xun] = prune(V, Vp, E, X, Xun, g);
                f = sampleFootholds(m, xmin, xmax, x0, xg, obs);
                [x, c] = generateSets(f, F, robot);
                F = [F, f];
                X = [X, x];
                C = [C, c];
                Xun = [Xun, size(X,2)-size(x,2)+1:size(X,2)];
                q = length(V) + length(Xun);
                Q = [Q, expand(1, V, Vp, E, X, Xun, C, radius(q, vol, n), g)];
            else
                Q = [Q, expand(Vin, V, Vp, E, X, Xun, C, radius(q, vol, n), g)];
                lastCost = minCost;
            end
            [infl, trunc] = updateParameters(q, minCost);
            Vcl = [];
            Vin = [];
            searchFinished = false;
        else
            Xp = C(:, Q(1,:));
            Xc = C(:, Q(2,:));
            if minCost == inf
                [~, I] = min(goalHeuristic(Xc, xg));
            else
                [~, I] = min(treeCost(1, Q(1,:), V, Vp, E) + edgeHeuristic(Xp, Xc) + ...
                    infl*goalHeuristic(Xc, xg));
            end
            edge = Q(:, I);
            Q(:,I) = [];
            if any(all(E(1:2,:) == edge, 1))
                if any(Vcl == edge(2))
                    Vin = [Vin, edge(2)];
                else
                    Q = [Q, expand(edge(2), V, Vp, E, X, Xun, C, ...
                        radius(q, vol, n), g)];
                    Vcl = [Vcl, edge(2)];
                end
            else
                xp = C(:, edge(1));
                xc = C(:, edge(2));
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
                                Q = [Q, expand(edge(2), V, Vp, E, X, Xun, C, ...
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
        minCost = iterCost(iter);
        if mod(iter, 500) == 0
            fprintf('\t%d: error = %.3f, f = %d, x = %d, q = %d\n', iter, ...
                iterCost(iter), size(F,2), size(X,2), size(Q,2));
            plotPath(E, V, Vp, X, C, Q, F, g, Xmin, Xmax, obs, robot)
            title(['Iteration: ', num2str(iter)]);
            drawnow();
            if iter > 3000 && minCost ~= inf
                break
            end
        end
    end
    
    path = getTree(1, 2, V, Vp, E);
    feet = indexFeet(X(:,V(:,path)), F);
    cost = minCost;
end

function plotPath(E, V, Vp, X, C, Q, F, g, xmin, xmax, obs, robot)
    clf;
    colormap gray
    hold on;
    axis equal;
    imagesc(xmin(2):xmax(2), xmin(1):xmax(1), obs');
%     for i = 1:size(Q, 2)
%         plot(X(1,Q(1:2,i)), X(2,Q(1:2,i)), 'r');
%     end
    for i = 1:size(E, 2)
%         plot(C(1,E(1:2,i)), C(2,E(1:2,i)), 'k');
    end
    plot(C(1,V), C(2,V), 'k.');
    for i = 1:4
        plot(F(1,:,i), F(2,:,i), '.');
    end
    plotRobot(F(:,1,:), 'g', robot)
    plotRobot(F(:,g,:), 'r', robot)
    
    path = getTree(1, 2, V, Vp, E);
    
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

function Eout = expand(Xi, V, Vp, E, X, Xun, C, r, g)
    r = inf;
    Eout = [];
    x0 = C(:,1);
    xg = C(:,g);
    best = min(treeCost(1, g, V, Vp, E));
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
        gtc = treeCost(1, c, V, Vp, E);
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

function F = createState(centroid, heading, robot)
    theta = heading + robot.theta0;
    F = centroid + robot.L0*[cosd(theta); sind(theta)];
    F = reshape(F, [], 1, 4);
end

function F = sampleFootholds(m, xmin, xmax, x0, xg, obs)
    F = rand(length(xmin), m, 4) .* (xmax - xmin) + xmin;
    t = (xg - x0)/norm(xg-x0);
    n = [-t(2); t(1)];
    R = [n, t];
    for k = 1:4
        F(:,:,k) = R*F(:,:,k) + x0;
        i = round(F(1,:,k)) + 1;
        j = round(F(2,:,k)) + 1;
        oob = i < 1 | j < 1 | i > size(obs,1) | j > size(obs,2);
        F(:,oob,k) = inf;
        i = max(1,min(i, size(obs,1)));
        j = max(1,min(j, size(obs,2)));
        F(:, ~obs(sub2ind(size(obs), i, j)), k) = inf;
    end
end

function [X, C] = generateSets(Fnew, F0, robot)
    F = [F0, Fnew];
    X = zeros(4,0);
    r = (robot.L0+robot.dL)*sqrt(2);
    for i1 = 1:size(F,2)
        p1 = F(:,i1,1);
        d2 = p1 - F(:,:,2);
        f2 = find(sum(d2.^2) < r.^2);
        for i2 = f2
            p2 = F(:,i2,2);
            d3 = cat(3, sqrt(1/2)*(p1 - F(:,:,3)), p2 - F(:,:,3));
            f3 = find(max(sum(d3.^2),[],3) < r.^2);
            for i3 = f3
                p3 = F(:,i3,3);
                d41 = p1 - F(:,:,4);
                d42 = sqrt(1/2)*(p2 - F(:,:,4));
                d43 = p3 - F(:,:,4);
                d4 = max(sum(d41.^2), max(sum(d42.^2), sum(d43.^2)));
                f4 = find(d4 < r.^2);
                if isempty(f4)
                    continue
                end
                x = [[i1;i2;i3].*ones(3, length(f4)); f4];
                X = [X, x];
            end
        end
    end
    X(:,max(X) <= size(F0, 2)) = []; % remove duplicates
    [C, ~, invalid] = getRobotPose(indexFeet(X, F), robot);
    X(:,invalid) = []; % remove invalid configurations
    C(:,invalid) = [];
end

function Fx = indexFeet(X, F)
    Fx = zeros(size(F,1), size(X,2), size(F,3));
    for i = 1:4
        Fx(:,:,i) = F(:,X(i,:),i);
    end
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