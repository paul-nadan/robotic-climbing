% TODO: back leg steps (reverse incline)
% TODO: plot forces
% TODO: add tail
% TODO: animation
% TODO: max/averaging results (return a single value)
% TODO: parameter sweep

ds = .02; % increment for arc length
animate = 1; % display robot states during sweep
display = 1; % display graph of sweep results at end

robot.L = [.071, .0987]; % leg lengths
robot.l = .2834*[0.5, 0.5]; % length of body segments
robot.w = .154; % width of body segments
robot.a = [-70, -60, -90;
            70,  90,  60]; % leg joint range of motion
robot.b = [-60; 60]; % body joint range of motion
robot.nfeet = 4; % number of feet
robot.wrist = 45*[-1; 1]; % wrist pitch range of motion
robot.y = .07; % horizontal distance from foot to shoulder

dstep = .05; % half the length of a step
lstep = sum(robot.l); % width of stance

terrain.angle = 90; % change in terrain incline (deg)
terrain.incline = 90; % average terrain incline relative to level ground (deg)
terrain.radius = .1; % minimum terrain radius of curvature
terrain.w = robot.w + 2*sum(robot.L);

% sweepAngles(f0, robot); return
options = optimoptions(@fmincon,'MaxFunctionEvaluations', 1e6,...
                                'Algorithm', 'interior-point',...
                                'Display', 'none',...
                                'CheckGradients', false,...
                                'SpecifyObjectiveGradient',true);
var = -90:5:90;
P = zeros(size(var))+nan;
C = zeros(size(var))+nan;
data = cell(2, length(var));
tic();
for i = 1:length(var)
    terrain.angle = var(i);
    fprintf('%d/%d: var = %d\n', i, length(var), var(i));
    [p, c, s, flags, frames] = sweep(robot, terrain, dstep, lstep, ds, ...
        options, animate, display);
    data{1,i} = p;
    data{2,i} = c;
    P(i) = min(p);
    C(i) = max(c);
    
    figure(3);
    clf;
    plot(var, C, 'r');
    hold on;
    plot(var, 1./P, 'k');
    xlabel('Var');
    ylabel('Cost');
    legend('CoM Displacement (m)', 'Force (N/N)');
end
fprintf('Time Elaspsed: %d hours, %d minutes, and %d seconds\n', ...
    floor(toc()/3600), floor(toc()/60), floor(mod(toc(),60)));

% for i = 1:length(var)
%     p = data{1,i};
%     P(i) = min(p);
% end
% figure(5);
% clf;
% plot(var, 1./P, 'k');
% hold on;
% for i = 1:length(var)
%     p = data{1,i};
%     p = mean([p(1:end-1); p(2:end)]);
%     P(i) = min(p);
% end
% plot(var, 1./P, 'b');
% for i = 1:length(var)
%     p = data{1,i};
%     p = mean([p(1:end-2); p(2:end-1); p(3:end)]);
%     P(i) = min(p);
% end
% plot(var, 1./P, 'c');
% for i = 1:length(var)
%     p = data{1,i};
%     p = max([p(1:end-1); p(2:end)]);
%     P(i) = min(p);
% end
% plot(var, 1./P, 'r');
% for i = 1:length(var)
%     p = data{1,i};
%     p = max([p(1:end-2); p(2:end-1); p(3:end)]);
%     P(i) = min(p);
% end
% plot(var, 1./P, 'm');
% xlabel('Var');
% ylabel('Force (N/N)');
% legend('Raw', '2-Mean', '3-Mean', '2-Min', '3-Min');

% figure(2);
% [costs, dfs] = loop(s, f0, step, robot, terrain, options);
% figure(3);
% hold on;
% plot(dfs, sqrt(costs), 'r.');

% plotTerrain3D(terrain, s);
% plotRobot3D(X, robot, 1, 1);

function [p, c, s, flags, frames] = sweep(robot, terrain, step, lstep, ds, options, animate, display)
    frames = struct('cdata',{},'colormap',{});
    f0 = -[0, step, 2*step, lstep+step/2, lstep+step*1.5];
    c = [];
    p = [];
    flags = [];
    X0 = [];
    offset = terrain.radius*abs(terrain.angle/2)*pi/180;
    s = -offset:ds:max(-f0)+offset;
    s_plot = f0(end)-offset:ds:-f0(end)+offset;
    for df = s
        f = f0 + df;
        [X,fval,flag,moment,payload,force] = optimize(f, robot, terrain, options, X0);
        X0 = X;
        c = [c, sqrt(fval)];
        p = [p, payload];
        flags = [flags, flag];

        if animate
            [~, ~, K, F] = getRobot(X, robot);
            guard = getGripperGuard(K, F, robot);
            figure(1);
            clf;
            plot(s2x(s_plot, terrain), s2z(s_plot, terrain), 'k');
            hold on; axis equal;
            plot(s2x(f, terrain), s2z(f, terrain), 'k.', 'markersize',20);
            n = [cosd(terrain.incline); -sind(terrain.incline)];
            goal = moment*n;
            plot(goal(1), goal(2), 'c.', 'markersize',20);
            g = [-sind(terrain.incline); -cosd(terrain.incline)]*robot.w(1)/2;
            l = [goal+g, goal-g];
            plot(l(1,:), l(2,:), 'c--');
            plot(guard(1,:), guard(3,:), 'm.', 'markersize',20);
            plotRobot(X, robot);
            drawnow();
            if isempty(frames)
                frames = getframe(gcf);
            else
                frames = [frames,getframe(gcf)];
            end
        end
    end
    if display
        if any(flags < 1)
            fprintf('Failure flag: %d\n', min(flags));
        end
        figure(2);
        clf;
        plot(s, c, 'r');
        hold on;
        plot(s, 1./p, 'k');
        xlabel('Arc Length (m)');
        ylabel('Cost');
        legend('CoM Displacement (m)','Force (N/N)');
        title(['Incline ',num2str(terrain.incline),'$^\circ$, Angle ',...
            num2str(terrain.angle),'$^\circ$, Radius ',...
            num2str(terrain.radius),'m']);
        drawnow();
    end
end

function [costs, dfs] = loop(s, f0, step, robot, terrain, options)
    df = 0;
    fmax = max(-f0);
    plotTerrain3D(terrain, s);
    costs = [];
    dfs = [];
    while df-step < fmax
        
        [X,fval,~,~] = optimize(f0+df, robot, terrain, options);
        plotRobot3D(X, robot, 1, 1);
        costs = [costs, fval]; dfs = [dfs, df];
        
        [X,fval,~,~] = optimize(f0+(fmax-df), robot, terrain, options);
        plotRobot3D(X, robot, -1, -1);
        costs = [costs, fval]; dfs = [dfs, (fmax-df)];
        
        df = df + step;
        
        [X,fval,~,~] = optimize(f0+df, robot, terrain, options);
        plotRobot3D(X, robot, -1, 1);
        costs = [costs, fval]; dfs = [dfs, df];

        [X,fval,~,~] = optimize(f0+(fmax-df), robot, terrain, options);
        plotRobot3D(X, robot, 1, -1);
        costs = [costs, fval]; dfs = [dfs, (fmax-df)];
        
        df = df + step
    end
end

function [X,fval,flag,moment,payload,force] = optimize(f, robot, terrain, options, X0)
    [moment, ~, ~] = getCoM(f, robot, terrain);
    x = s2x(f, terrain);
    z = s2z(f, terrain);
    ang = atan2d(z(1)-z(end), x(1)-x(end));
    a0 = repmat([0; robot.a(1,2); -robot.a(1,2)], 1, size(f,2));
    if nargin < 5 || isempty(X0)
        X0 = [mean(x([3,4])); mean(z([3,4])); ang; 0; reshape(a0, [], 1)];
    end
    lb = [-inf, -inf, -inf, robot.b(1,:), repmat(robot.a(1,:),1,robot.nfeet+1)];
    ub = [inf, inf, inf, robot.b(2,:), repmat(robot.a(2,:),1,robot.nfeet+1)];
    [X,fval,flag,~] = fmincon(@(X)cost(X,moment,robot,terrain), X0, ...
        [],[],[],[], lb,ub, @(X)constraints(X,x,z,robot,terrain), options);
%     if flag == -2
        X0 = [mean(x([3,4])); mean(z([3,4])); ang; 0; reshape(a0, [], 1)];
        [X2,fval2,flag2,~] = fmincon(@(X)cost(X,moment,robot,terrain), X0, [],[],[],[], lb,ub, ...
            @(X)constraints(X,x,z,robot,terrain), options);
        if flag2 > 0 && (fval2 < fval - 1e-4 || flag <= 0)
            X = X2;
            fval = fval2;
            flag = flag2;
        end
%     end
    moment = X(1:2)'*[cosd(terrain.incline); -sind(terrain.incline)];
    [~, force, payload] = getCoM(f, robot, terrain, moment);
end

function z = x2z(x, terrain)
    r = terrain.radius;
    a = terrain.angle;
    z = tand(a/2).*abs(x);
    curve = abs(x) < r * sind(abs(a/2));
    z(curve) = (r*sqrt(1-x(curve).^2/r^2) - r/cosd(a/2))*sign(-a);
end

function x = s2x(s, terrain)
    r = terrain.radius;
    a = terrain.angle;
    x = (s - sign(s)*r * abs(a)/2 * pi/180).*cosd(a/2) + sign(s)*r*sind(abs(a)/2);
    curve = abs(s) < r * abs(a)/2 * pi/180;
    x(curve) = r*sin(s(curve)/r);
end

function z = s2z(s, terrain)
    z = x2z(s2x(s, terrain), terrain);
end

function a = s2a(s, terrain)
    e = 1e-6;
    a = atan2d(s2z(s, terrain) - s2z(s-e, terrain),...
               s2x(s, terrain) - s2x(s-e, terrain));
end

function [foot, knee] = forward(a1, a2, a3, L)
    r = [L(1).*cosd(a2); -L(2).*sind(a2 + a3) + L(1).*cosd(a2)];
    z = [-L(1).*sind(a2); -L(2).*cosd(a2 + a3) - L(1).*sind(a2)];
    x = r.*sind(a1);
    y = r.*cosd(a1);
    foot = [x(2,:); y(2,:); z(2,:)];
    knee = [x(1,:); y(1,:); z(1,:)];
end

function [c, ceq] = constraints(X, x, z, robot, terrain)
    [B, S, K, F] = getRobot(X, robot);
    ceq = reshape(F - [x;x*0+robot.y;z], [], 1);
    s = (0:.02:1)';
    B1 = reshape(B(1, 1:end-1).*s + B(1, 2:end).*(1-s), [], 1);
    B3 = reshape(B(3, 1:end-1).*s + B(3, 2:end).*(1-s), [], 1);
    K1 = reshape(K(1, :).*s + S(1, :).*(1-s), [], 1);
    K3 = reshape(K(3, :).*s + S(3, :).*(1-s), [], 1);
    s = (.01:.02:1)';
    F1 = reshape(K(1, :).*s + F(1, :).*(1-s), [], 1);
    F3 = reshape(K(3, :).*s + F(3, :).*(1-s), [], 1);
    guard = getGripperGuard(K, F, robot);
    c = -[B3 - x2z(B1, terrain);
          K3 - x2z(K1, terrain);
          F3 - x2z(F1, terrain);
          guard(3,:)' - x2z(guard(1,:), terrain)'];
end

function [c, g] = cost(X, moment, ~, terrain)
    com = X(1:2);
    n = [cosd(terrain.incline); -sind(terrain.incline)];
    t = [sind(terrain.incline); cosd(terrain.incline)];
%     goal = mean([x([2,4,5]); z([2,4,5])],2);
    goal = moment*n;
    d = com-goal;
    r = 0;
    c = (d'*n)^2 + r*(d'*t)^2;
    g = 2*n'*d*n + r*2*t'*d*t;
    g = [g;zeros(size(X(3:end)))];
end

function [B, S, K, F] = getRobot(X, robot)
    nb = length(robot.l)+2;
    na = size(robot.a, 2)*(robot.nfeet+1);
    b = X(1:nb);
    a = reshape(X(nb+1:nb+na), [], robot.nfeet+1);
    com = [b(1);0;b(2)];
    b1 = [0;0;0];
    b2 = b1 + robot.l(1)*[-cosd(b(3)); 0; -sind(b(3))];
    b3 = b2 + robot.l(2)*[-cosd(b(3)+b(4)); 0; -sind(b(3)+b(4))];
    B = [b1, b2, b3];
    B = B + com - mean(B(:,[1,2,2,3]),2);
    S = B(:,[1,1,1,3,3]);
    S(2,:) = 0;
    [F, K] = forward(a(1,:), a(2,:), a(3,:), robot.L);
    F = [getR3(b(3))*F(:,1:3), getR3(b(3)+b(4))*F(:,4:5)] + S;
    K = [getR3(b(3))*K(:,1:3), getR3(b(3)+b(4))*K(:,4:5)] + S;
end

function guard = getGripperGuard(K, F, robot)
    d = F-K;
    d(2,:) = 0;
    d = d./sqrt(sum(d.^2,1));
    d1 = getR3(robot.wrist(1) - 90)*d;
    d2 = getR3(robot.wrist(2) + 90)*d;
    guard = [F + 0.005*d1, F + 0.005*d2];
end

function plotRobot(X, robot)
    [B, S, K, F] = getRobot(X, robot);
    plot(B(1,:), B(3,:), 'r');
    plot(F(1,:), F(3,:), 'r.', 'markersize', 20);
    plot([F(1,:); K(1,:)], [F(3,:); K(3,:)], 'b');
    plot([S(1,:); K(1,:)], [S(3,:); K(3,:)], 'b');
    com = mean(B(:,[1,2,2,3]),2);
    plot(com(1), com(3), 'g.', 'markersize', 20);
end

function plotTerrain3D(terrain, s)
    clf;
    x = s2x(s, terrain);
    y = [-terrain.w/2,terrain.w/2];
    Z = s2z(s, terrain).*ones(size(y'));
    surface(x, y, Z);
    axis equal;
    hold on;
end

function plotRobot3D(X, robot, mirror, reverse)
    [B, S, K, F] = getRobot(X, robot);
    com = mean(B(:,[1,2,2,3]),2);
    offset = [0; robot.w/2; 0];
    B = B.*[1;mirror;1] + offset*mirror;
    S = [S(:,[1,3,4]) + offset, S(:,[2,5]).*[1;-1;1] - offset].*[1;mirror;1];
    K = [K(:,[1,3,4]) + offset, K(:,[2,5]).*[1;-1;1] - offset].*[1;mirror;1];
    F = [F(:,[1,3,4]) + offset, F(:,[2,5]).*[1;-1;1] - offset].*[1;mirror;1];
    Bx = [B(1,1:end-1); B(1,2:end); B(1,2:end); B(1,1:end-1)];
    By = [B(2,1:end-1); B(2,2:end); -B(2,2:end); -B(2,1:end-1)];
    Bz = [B(3,1:end-1); B(3,2:end); B(3,2:end); B(3,1:end-1)];
    patch(Bx*reverse, By, Bz, 'r');
    plot3(F(1,:)*reverse, F(2,:), F(3,:), 'r.', 'markersize', 20);
    plot3([F(1,:); K(1,:)]*reverse, [F(2,:); K(2,:)], [F(3,:); K(3,:)], 'b');
    plot3([S(1,:); K(1,:)]*reverse, [S(2,:); K(2,:)], [S(3,:); K(3,:)], 'b');
    plot3(com(1)*reverse, com(2), com(3), 'g.', 'markersize', 20);
end

function R = getR3(theta)
    R = [cosd(theta), 0,-sind(theta);
         0,           1, 0;
         sind(theta), 0, cosd(theta)];
end

function R = getR2(theta)
    R = [cosd(theta),-sind(theta);
         sind(theta), cosd(theta)];
end

function V = getSkew(v)
    V = [0,    -v(3),  v(2);
         v(3),  0,    -v(1);
        -v(2),  v(1),  0];
end

function [moment, force, payload] = getCoM(f, robot, terrain, moment)
    if nargin < 4
        moment = [];
    end
    x = s2x(f([2,4,5]), terrain);
    y = (robot.y + robot.w/2)*[1,-1,1]; % FL, RR, RL
    z = s2z(f([2,4,5]), terrain);
    X = getR3(terrain.incline)*[x;y;z];
    incline = s2a(f([2,4,5]), terrain) + terrain.incline - 90;
    % x = N, y = B, z = T
    % [N1 B1 T1 N2 B2 T2 N3 B3 T3 moment cost]
    % [N B T yaw pitch roll]
    F0 = [0;0;1/3; 0;0;1/3; 0;0;1/3; 0; 1/3];
%     F0(1:9) = [0.2943
%     0.0207
%     0.6309
%     0.0815
%     0.5508
%     0.4184
%    -0.3758
%    -0.5714
%    -0.0493];
    Aeq = [eye(3,3), eye(3,3), eye(3,3), zeros(3,2);
           getSkew(X(:,1)), getSkew(X(:,2)), getSkew(X(:,3)), [0 0; 1 0; 0 0]];
    beq = [0;0;1; 0;0;0];
    
%     lb1 = [-inf,   0,-inf,-inf,-inf,-inf,-inf,   0,-inf,-inf,-inf];
%     ub1 = [ inf, inf, inf, inf,   0, inf, inf, inf, inf, inf, inf];
%     
%     lb2 = [-inf,-inf,-inf,-inf,  -0,-inf,-inf,-inf,-inf,-inf,-inf];
%     ub2 = [ inf,   0, inf, inf, inf, inf, inf,   0, inf, inf, inf];
    
    if ~isempty(moment)
        Aeq = [Aeq; zeros(1,9), 1, 0];
        beq = [beq; moment];
    end
    options = optimoptions(@fmincon, 'Display', 'none', ...
                                     'SpecifyObjectiveGradient',true);
    [F,cost,~,~] = fmincon(@forceCost, F0, [],[], Aeq,beq, [],[], ...
        @(F)forceConstraints(F, incline), options);
    force = reshape(F(1:end-2), 3, []);
    moment = F(end-1);
    payload = 1/cost;
end

function [c, ceq] = forceConstraints(F, incline)
    cost = F(end);
    F = reshape(F(1:end-2), 3, []);
    f1 = getR3(incline(1))*F(:,1);
    f2 = getR3(incline(2))*F(:,2);
    f3 = getR3(incline(3))*F(:,3);
    F = [f1 f2 f3];
    F(1,:) = min(0, F(1,:));
    c = [sqrt(sum((F.*[1;1;1]).^2))' - cost;
         -F(1,:)' - tand(20)*sqrt(F(2,:).^2 + F(3,:).^2)'];
    ceq = [];
end

function [c, g] = forceCost(F)
    c = F(end);
    g = zeros(size(F));
    g(end) = 1;
end

function sweepAngles(f, robot)
    terrain.angle = 0; % change in terrain incline (deg)
    terrain.radius = 0; % minimum terrain radius of curvature
    m = [];
    p = [];
    inclines = -180:1:180;
    for incline = inclines
        terrain.incline = incline;
        [moment, ~, payload] = getCoM(f, robot, terrain);
        m = [m, moment];
        p = [p, payload];
    end
    figure(5);
    clf;
    plot(inclines, m, 'r');
    hold on;
    plot(inclines, 1./p, 'k');
    xlabel('Incline ($^\circ$)')
    ylabel('Cost');
    legend('CoM Displacement (m)','Force (N/N)');
    title('Flat Terrain (Ideal)');
end