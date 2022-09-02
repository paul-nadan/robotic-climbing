figure(1);

x0 = [1; 1]; % initial state
xg = [7, 9; 9, 7]; % goal state
xmin = [0; 0]; % state space lower bound
xmax = [10; 10]; % state space upper bound
iters = 20; % iterations
m = 100; % samples per batch

n = size(x0, 1); % state space dimension
vol = prod(xmax-xmin); % volume of free space
X = [x0, xg]; % indexed array of states nxN
g = 2:length(X); % goal indices
Xun = g; % unconnected nodes 1xN
V = 1; % RGG vertices 1xN
E = zeros(3,0); % RGG edges + costs 3xN
q = length(V) + length(Xun); % number of samples
infl = inf; % inflation factor
trunc = inf; % truncation factor
Vcl = []; % closed (already explored) vertices 1xN
Vin = []; % inconsistent vertices 1xN
Q = expand(1, V, E, X, Xun, inf, g); % edge queue 2xN
iterCost = zeros(1, iters) + nan;

obs = rand(xmax(1)-xmin(1)+1, xmax(2)-xmin(2)+1) > 2/5;
obs(x0(1)+1, x0(2)+1) = 1;
for i = 1:size(xg, 2)
    obs(xg(1,i)+1, xg(2,i)+1) = 1;
end
    
searchFinished = true;
lastCost = inf;
for iter = 1:iters
    if searchFinished
        if lastCost == min(iterCost)
%             [V, E, Xun] = prune(V, E, X, Xun, g);
            X = [X, sample(m, min(iterCost), x0, xg, xmin, xmax, obs)];
            Xun = [Xun, length(X)-m+1:length(X)];
            q = length(V) + length(Xun);
            Q = [Q, expand(1, V, E, X, Xun, radius(q, vol, n), g)];
        else
            Q = [Q, expand(Vin, V, E, X, Xun, radius(q, vol, n), g)];
            lastCost = min(iterCost);
        end
        [infl, trunc] = updateParameters(q, min(iterCost));
        Vcl = [];
        Vin = [];
        searchFinished = false;
    else
        Xp = X(:, Q(1,:));
        Xc = X(:, Q(2,:));
        [~, I] = min(treeCost(1, Q(1,:), V, E) + edgeHeuristic(Xp, Xc) + ...
            infl*goalHeuristic(Xc, xg));
        edge = Q(:, I);
        Q(:,I) = [];
        if any(all(E(1:2,:) == edge, 1))
            if any(Vcl == edge(2))
                Vin = [Vin, edge(2)];
            else
                Q = [Q, expand(edge(2), V, E, X, Xun, ...
                    radius(q, vol, n), g)];
                Vcl = [Vcl, edge(2)];
            end
        else
            xp = X(:, edge(1));
            xc = X(:, edge(2));
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
                            Q = [Q, expand(edge(2), V, E, X, Xun, ...
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
    if mod(iter, 10) == 0
        fprintf('%d: error = %.3f\n', iter, iterCost(iter));
        plotPath(E, V, X, Q, x0, xg, xmin, xmax, obs);
        title(['Iteration: ', num2str(iter)]);
        drawnow();
    end
end

function plotPath(E, V, X, Q, x0, xg, xmin, xmax, obs)
    clf;
    colormap gray
    hold on;
    axis equal;
    imagesc(xmin(2):xmax(2), xmin(1):xmax(1), obs');
    for i = 1:size(Q, 2)
        plot(X(1,Q(1:2,i)), X(2,Q(1:2,i)), 'r');
    end
    for i = 1:size(E, 2)
        plot(X(1,E(1:2,i)), X(2,E(1:2,i)), 'b');
    end
    for i = 1:length(X)
        plot(X(1,:), X(2,:), 'r.');
    end
    for i = 1:length(V)
        plot(X(1,V(i)), X(2,V(i)), 'b.');
    end
    path = getTree(1, 2, V, E);
    plot(X(1,V(path)), X(2,V(path)), 'c-', 'linewidth', 5);
    path = getTree(1, 3, V, E);
    plot(X(1,V(path)), X(2,V(path)), 'c-', 'linewidth', 5);
    plot(x0(1,:), x0(2,:), 'g.', 'markersize', 25);
    plot(xg(1,:), xg(2,:), 'r.', 'markersize', 25);
    xlim([xmin(1), xmax(1)]);
    ylim([xmin(2), xmax(2)]);
end

function Eout = expand(Xi, V, E, X, Xun, r, g)
    Eout = [];
    x0 = X(:,1);
    xg = X(:,g);
    best = min(treeCost(1, g, V, E));
    for p = Xi
        Eout = [Eout, E(1:2, E(1,:) == p)];
        Xall = [Xun, V];
        xp = X(:, p);
        d = xp - X(:,Xall);
        ghatp = startHeuristic(x0, xp);
        c = Xall(sum(d.^2, 1) <= r^2);
        gtc = treeCost(1, c, V, E);
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
        infl = 1e6;
    end
end

function x = sample(m, cost, x0, xg, xmin, xmax, obs)
    x = rand(length(xmin), m) .* (xmax - xmin) + xmin;
    for i = 1:10
        I = startHeuristic(x0, x) + goalHeuristic(x, xg) > cost;
        I = I | isObstacle(x, obs);
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
    chat = vecnorm(xp-xc);
end

function c = edgeCost(xp, xc, obs)
    c = vecnorm(xp-xc);
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