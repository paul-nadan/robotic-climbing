close all;
set(0,'defaultfigureposition',[400 100 450 750])
% clearvars -except {get* body solveQP solveQP_old}
theta = [ones(1, robot.n-robot.tail).*[0; -45; 30], [0; 0; -5]];    % Joint angles (deg)
x = [0; 0; 0.05];      % Centroid position (m)
R = eye(3);         % Orientation (rotation matrix)
g = 9.81;           % Gravity (m/s^2)
m = 2;              % mass (kg)
bounds = [repmat([-70 70; -70 90; -60 60], robot.n-robot.tail, 1); -60 60; -80 80];



[~, corner] = robot.getJoints(theta);

r0 = reshape(robot.getJoints(theta), [], 1);
x(3) = x(3)-r0(3, 1);
r0(1:3) = r0(1:3) + [0; .15; 0];

% r0 = reshape(corner, [], 1);

rng(3);
theta(:, 1:4) = theta(:, 1:4) + 2*(rand(3, 4)-.5) * 45;
theta = vec2theta(max(min(theta2vec(theta), bounds(:, 2)), bounds(:, 1)));
% theta(1, 1:4) = theta(1, 1:4) + 45;
feet = robot.getJoints(theta);

mode = [1 1 1 1];
offset = 0;
yaw = 0;

% [force, margin] = solveQP(feet, offset*[1; -1; 1; -1], ...
%                 [0; m*g; 0; -m*g*h; 0; 0], mode, 20, yaw, 2, 20);
% torque = getTorques(feet, force);

r = reshape(feet, [], 1);
[X, Y] = meshgrid([-1, 1]*1, [-1, 1]*1);
Z = 0*X;
% r0 = zeros(size(r));
X0 = R*feet+x;
wmax = .5;
options = optimoptions('quadprog', 'Display', 'off');

frames = struct('cdata',{},'colormap',{});
tic()
for i = 1:500
    if mod(i, 10) == 1
        cla;
%         surf(X, Z, -Y);
        drawrobot(x, R, theta, robot);
        xlim([-.4, .4]);
        ylim([-.3, .3]);
        zlim([-1, 1]/2);
%         hold off
        drawnow;
        if isempty(frames)
            frames = [getframe(gcf)];
        else
            frames = [frames, getframe(gcf)];
        end
    end

    G = robot.getGraspMap(feet);
    J = robot.getJacobian(theta);
%     Gt = robot.getGraspMap(feet(:, end));
    h = x(3);
%     [dtheta, dr, twist] = recenter(G, J, theta, r(1:end-3), r0(1:end-3));
    [dtheta, dr, twist, cost] = recenterQP(G, J, theta, bounds, wmax, r, r0, h, options);
    theta = theta + vec2theta(dtheta);
    x = x - twist(1:3);
    R = vrrotvec2mat([twist(4:6); norm(twist(4:6))])'*R;
%     dr1 = robot.getJoints(theta) - feet;
%     dr1(:, 1:4) - reshape(dr, 3, [])
    feet = robot.getJoints(theta);
    r = reshape(feet, [], 1);
    if norm(dtheta) < 1e-5
        i
        break
    end
end
toc()
dX = R*feet+x - X0

clf;
drawrobot(x, R, theta, robot);



% drawrobot(x, R, theta, getJoints, body(3));
% drawforces([feet, x + [0; 0; h]], [force, [0; -m*g; 0]]);
% title(['Margin: ', num2str(margin, '%.3f'), ' N']);
% margin

%% Controls Functions

function [dtheta, dr, twist] = recenter(G, J, theta, r, r0)
    W = diag([1 1 1 1 1 1]);
    dr = (r0 - r)/10;
    dr(4:end) = G(:, 4:end-3)'*W*pinv(G(:, 4:end-3))'*dr(4:end);
    if max(abs(dr)) > 0.001
        dr = dr / max(abs(dr)) * .001;
    end
    twist = pinv(G(:, 4:end-3))'*dr(4:end);
    dr = [dr; -[0; 0; 1].*G(:, end-2:end)'*twist];
    dtheta = pinv(J)*dr;
%     dtheta(13) = 0;
%     (eye(9) - G(:, 4:end-3)'*pinv(G(:, 4:end-3))')*dr(4:end-3)
end

function [dtheta, dr, twist, cost] = recenterQP(G, J, theta, bounds, wmax, r, r0, h, options)
    theta = theta2vec(theta);
    n = size(J, 1);
    
    swing = [1,2,3];
    
    Id = eye(n);
    Id(swing, :) = 0;
    J([13, 14], :) = 0;
    
%     Jinv = pinv(J([1:12, 15], :));
%     Jinv = [Jinv(:, 1:12), zeros(size(J, 2), 2), Jinv(:, end)];
    
    Jinv = pinv(J);
    Ginv = pinv(G);
    Ginv(swing, :) = 0;

    H = eye(n);
    H(1:3, 1:3) = H(1:3, 1:3) * 10;
    H(13:14, 13:14) = 0;
    f = -2*(r0 - r);
    f(1:3) = f(1:3) * 10;
    f(13:14) = 0;
    f([6, 9, 12]) = f([6, 9, 12]) - sign(h);
    h = max(0, h);
    
    A = [-Jinv;
          Jinv;
          Ginv(:, 3)'];
    b = [-max(bounds(:, 1) - theta, -wmax);
          min(bounds(:, 2) - theta, wmax);
          h];
    
    Aeq = Id - G'*Ginv';
    beq = zeros(n, 1);
    
    [dr, cost, flag] = quadprog(H, f, A, b, Aeq, beq, [], [], [], options);
    dr = dr/2;
    if flag < 0
        flag
        cost = 0;
        dr
        dr = zeros(n, 1);
        J
        Jinv
        G
        Ginv
    end
    
    dtheta = Jinv*dr;
%     dtheta = [dtheta; 0];
    twist = Ginv'*dr;
%     cost + (r0-r)'*(r0-r);
%     cost
end

function theta = theta2vec(theta)
    theta = reshape(theta, [], 1);
    theta = theta([1:end-3, end-1:end]);
end

function theta = vec2theta(theta)
    theta = [theta(1:end-2); 0; theta(end-1:end)];
    theta = reshape(theta, 3, []);
end

%% Drawing Functions

% Draw the robot
function drawrobot(x, R, a, robot)
    c = [1, 0, 0];
    cbody = 0.9*c;
    cleg = 0.6*c;
    cjoint = 0.3*c;
    rleg = 0.01;
    rjoint = 0.02;
    [~, corner, shoulder, knee, foot, base, tail] = robot.getJoints(a);
    body1 = [corner(:, 1:2),  corner(:, 1:2).* [1; 0; 0]];
    body2 = [corner(:, end-1:end),  corner(:, end-1:end).* [1; 0; 0]];
    Rplot = vrrotvec2mat([1, 0, 0, pi/2]);
    n1 = cross(body1(:, 1) - body1(:,3), body1(:, 1) - body1(:,2));
    n2 = cross(body2(:, 1) - body2(:,3), body2(:, 1) - body2(:,2));
    offset1 = Rplot*R*n1/norm(n1)*robot.h/2;
    offset2 = Rplot*R*n2/norm(n2)*robot.h/2;
    foot = Rplot*(R*foot+x);
    knee = Rplot*(R*knee+x);
    shoulder = Rplot*(R*shoulder+x);
    corner = Rplot*(R*corner+x);
    body1 = Rplot*(R*body1+x);
    body2 = Rplot*(R*body2+x);
    base = Rplot*(R*base+x);
    tail = Rplot*(R*tail+x);
    prism(body1(:, [1 2 4 3]) + offset1, ...
          body1(:, [1 2 4 3]) - offset1, cbody);
    prism(body2(:, [1 2 4 3]) + offset2, ...
          body2(:, [1 2 4 3]) - offset2, cbody);
%     cylinder(body1(:, 3), body1(:, 4), robot.h/2*sqrt(2), cleg);
    axis equal;
    for i = 1:size(foot, 2)
        cylinder(corner(:, i), shoulder(:, i), rleg, cleg);
        cylinder(shoulder(:, i), knee(:, i), rleg, cleg);
        cylinder(knee(:, i), foot(:, i), rleg, cleg);
        ball(corner(:, i), rjoint, cjoint);
        ball(shoulder(:, i), rjoint, cjoint);
        ball(knee(:, i), rjoint, cjoint);
        ball(foot(:, i), rjoint, cjoint);
    end
    if size(tail, 2) > 0
        cylinder(base(:, end), tail(:, end), rleg, cleg);
        ball(base(:, end), rjoint, cjoint);
        ball(tail(:, end), rjoint, cjoint);
    end
end

% Draw the force vectors
function drawforces(r, f)
    R = vrrotvec2mat([1, 0, 0, pi/2]);
    r = R*r;
    f = R*f/40;
    quiver3(r(1,:), r(2,:), r(3,:), ...
            f(1,:), f(2,:), f(3,:), ...
            0, 'b', 'linewidth', 5);
end

% Plot a prism given two opposing faces with equal numbers of vertices
function prism(face1, face2, varargin)
    fill3(face1(1,:), face1(2,:), face1(3,:), varargin{:});
    hold on;
    fill3(face2(1,:), face2(2,:), face2(3,:), varargin{:});
    for i = 1:size(face1, 2)-1
        face = [face1(:, i), face1(:, i+1), face2(:, i+1), face2(:, i)];
        if size(face1, 2) >= 10
            fill3(face(1,:), face(2,:), face(3,:), varargin{:}, ...
                'EdgeColor', 'none');
        else
            fill3(face(1,:), face(2,:), face(3,:), varargin{:});
        end
    end
    face = [face1(:, 1), face1(:, end), face2(:, end), face2(:, 1)];
    if size(face1, 2) >= 10
        fill3(face(1,:), face(2,:), face(3,:), varargin{:}, ...
            'EdgeColor', 'none');
    else
        fill3(face(1,:), face(2,:), face(3,:), varargin{:});
    end
end

% Plot a cylinder given to end points and a radius
function cylinder(center1, center2, radius, color, varargin)
    x = linspace(0, 360, 10);
    R = vrrotvec2mat(vrrotvec([0; 0; 1], center2 - center1));
    face1 = R*radius*[cosd(x); sind(x); zeros(size(x))] + center1;
    face2 = face1 + center2 - center1;
    
    X = [face1(1, :); face2(1, :)];
    Y = [face1(2, :); face2(2, :)];
    Z = [face1(3, :); face2(3, :)];
    C = reshape(color, 1, 1, 3).*ones(size(X));
    surface(X, Y, Z, C, varargin{:}, 'EdgeColor', 'none');
end

% Plot a cylinder given to end points and a radius
function ball(center, radius, color, varargin)
    [X, Y, Z] = sphere();
    C = reshape(color, 1, 1, 3).*ones(size(X));
    for i = 1:size(center, 2)
        surface(X*radius + center(1, i), ...
                Y*radius + center(2, i), ...
                Z*radius + center(3, i), ...
                C, varargin{:}, 'EdgeColor', 'none');
    end
end