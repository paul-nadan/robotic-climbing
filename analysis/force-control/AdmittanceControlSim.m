% x is right, y is forward, z is up (relative to the wall)
% state = [pos; quat; FL; FR; RL; RR; tail; vel; angvel]

set(0,'defaultfigureposition',[400 100 900 750])
close all;

k.period = 0.02;    % Controller update period (s)
k.dt = 0.001;       % Simulation time step (s)
k.tmax = 6;         % Simulation duration (s)
k.ff = 1;           % Animation playback speed multiplier

k.kp = .01;         % Controller gain (m/N)
k.vmax = 1;         % Maximum controller velocity output (m/s)
k.x0 = 0.15;        % Leg length in sprawl posture

k.pfnoise = 0e-1;      % Process noise - force (N)
k.mfnoise = 0;      % Measurement noise - force (N)
k.mrnoise = 0e-3;   % Measurement noise - position (m)

k.w = 0.154;        % Robot width
k.l = 0.283;        % Robot length
k.h = 0.054;        % Robot height
k.tail = 0.350;     % Robot tail length
k.m = 2.16;         % Robot mass (kg)
k.jx = 1;           % Robot x moment of inertia (kg-m^2)
k.jy = 1;           % Robot y moment of inertia (kg-m^2)
k.jz = 1;           % Robot z moment of inertia (kg-m^2)
k.kx = 1000;        % Gripper x stiffness (N/m)
k.ky = 1000;        % Gripper y stiffness (N/m)
k.kz = 1000;        % Gripper z stiffness (N/m)
k.kt = k.kz;        % Tail z stiffness (N/m)
k.bx = 20;          % Gripper x damping (N-s/m)
k.by = 20;          % Gripper y damping (N-s/m)
k.bz = 20;          % Gripper z damping (N-s/m)
k.bt = k.bz;        % Tail z stiffness (N/m)
k.g = [0; -9.8; 0]; % Gravitational acceleration (m/s^2)

k.J = diag([k.jx, k.jy, k.jz]);
k.Jinv = inv(k.J);
k.K = diag([k.kx, k.ky, k.kz]);
k.Kt = diag([0, 0, k.kt]);
k.B = diag([k.bx, k.by, k.bz]);
k.Bt = diag([0, 0, k.bt]);

p0 = zeros(3, 1);
q0 = [1; 0; 0; 0];
v0 = zeros(3, 1);
w0 = zeros(3, 1);
r0 = p0 + [-1, 1, -1, 1, 0; 
           1, 1, -1, -1, -(2*k.tail + k.l)/k.l;
           -1, -1, -1, -1, -1] .* [k.w/2 + k.x0; k.l/2; k.h*2];
x0 = [p0; q0; reshape(r0, [], 1); v0; w0];

r0 = r0 + (rand(3, 5) - 0.5) * .1; % Inject error
[T, X, U, F, Fgoal] = simulate(x0, r0, k);

frames = animate(T, X, F, k);

plotforces(T, F, Fgoal);

function [T, X, U, F, Fgoal] = simulate(x0, r0, k)
    N = ceil(k.tmax / k.period);    % Number of controller updates
    n = ceil(k.period / k.dt);      % Number of timesteps per update
    T = zeros(1, n, N);             % Times
    X = zeros(length(x0), n, N);    % States
    U = zeros(15, n, N);            % Inputs
    F = zeros(15, n, N);            % Actual contact forces
    Fgoal = zeros(15, n, N);        % Desired contact forces
    t0 = k.dt;                      % Current time
    f0 = zeros(15, 1);              % Current contact forces
    
    for i = 1:N
        [p, q, r, ~, ~] = unpack(x0);
        
        % Compute contact mode
        contacts = gait(t0);
        for j = 1:4
            if ~contacts(j)
                r0(:, j) = NaN;
            elseif any(isnan(r0(:, j)))
                
                r0(:, j) = r(:, j);
            end
        end
        
        % Compute control input
        fhat = q2R(q)'*reshape(f0, 3, 5);
        rhat = q2R(q)'*(reshape(r, 3, 5) - p);
        fhat = random('Normal', fhat, k.mfnoise);
        fhat([2,5,8,11]) = fhat([2,5,8,11]) - [0 5 0 -5];
        rhat = random('Normal', rhat, k.mrnoise);
        [u, fgoal] = control(t0, rhat, fhat, contacts, k);
        
        % Simulate dynamics
        tspan = t0:k.dt:t0+k.dt*(n-1);
        [t, x] = ode45(@(t, x)update(t, x, u, r0, k), tspan, x0);
        x = x';
        t0 = t(end) + k.dt;
        x0 = x(:, end);
        
        % Recompute forces
        f = zeros(15, n);
        for j = 1:n
            [~, f0] = update(t(j), x(:, j), u, r0, k);
            f(:, j) = reshape(fhat, [], 1);
        end
        
        % Store results
        T(:, :, i) = t;
        X(:, :, i) = x;
        U(:, :, i) = repmat(reshape(u, [], 1), 1, n);
        Fgoal(:, :, i) = repmat(reshape(fgoal, [], 1), 1, n);
        F(:, :, i) = f;
    end
    T = reshape(T, size(T, 1), []);
    X = reshape(X, size(X, 1), []);
    U = reshape(U, size(U, 1), []);
    F = reshape(F, size(F, 1), []);
    Fgoal = reshape(Fgoal, size(Fgoal, 1), []);
end

function [dx, f] = update(~, x, u, r0, k)
    [p, q, r, v, w] = unpack(x);
    G = grasp(r - p);
    
    % Kinematics
    dp = v;
    dq = q2L(q)*[0;w]/2;
    dr = reshape(G'*[v; w], 3, 5) + q2R(q)*u;
    
    % Dynamics
    d = r0 - r;
    f = zeros(3, 5);
    for i = 1:4
        if ~any(isnan(r0(:, i)))
            f(:, i) = k.K * d(:, i) - k.B * dr(:, i);
        end
    end
    f(:, 5) = k.Kt * max(0, d(:, 5)) - k.Bt * dr(:, 5);

    wrench = G*reshape(f, [], 1);
    dv = 1/k.m * wrench(1:3) + k.g;
    dw = k.Jinv * wrench(4:6);
    
    dx = [dp; dq; reshape(dr, [], 1); dv; dw];
    f = reshape(f, [], 1);
end

% Compute foot displacements in body frame given state in body frame
function [u, fgoal] = control(t, r, f, contacts, k)
    r(3,:) = -0.0;
    G = grasp(r);
    wrench = G(:, [1:12, 15])*reshape(f([1:12, 15]), [], 1);
    G = G(:, 1:end-3);
%     fgoal = sum(f, 2);
%     fgoal = [fgoal*0, fgoal/4, fgoal/2, fgoal/4, 0*fgoal];
    
    fgoal = setpoints(wrench, contacts, k);
    fgoal(2,1:4) = fgoal(2,1:4);% - [0 0 0 50];

    u = -k.kp * (fgoal - f);
    u = reshape(u, [], 1);
    for i = 1:4
        G(:, i*3-2:i*3) = G(:, i*3-2:i*3)*contacts(i);
        u(i*3-2:i*3) = u(i*3-2:i*3)*contacts(i);
    end
    u(1:end-3) = u(1:end-3) - G'*pinv(G') * u(1:end-3);
%     pinv(G')*u(1:end-3)
%     G*u(1:end-3)
    u = reshape(u, 3, 5);
%     u = u + [0; -0.01; 0];
    u = min(max(u, -k.vmax), k.vmax);
    u(1:2, 5) = 0;
end

function f = setpoints(wrench, contacts, k)
    fmin = min(2, wrench(2)/4);
    r = k.l/2;
    L = k.tail/r + 1;
    g = wrench(2) - 4*fmin;
    pitch = -wrench(4);
    
    stancePitch = -0.5*(L-1)/(L+1) + 0.5;
    stance = [0 0 0 0 0; 0.5, 0.5, 0, 0, 0; -0.5, -0.5, 0, 0, 1].*...
        [0; g; pitch*stancePitch/r];
    
    swingPitch = 1/L;
    swing1 = [0 0 0 0 0; 0, 0.5, 0.5, 0, 0; 0, -0.5, -0.5, 0, 1].*...
        [0; g; pitch*swingPitch/r];
    swing2 = [0 0 0 0 0; 0.5, 0, 0, 0.5, 0; -0.5, 0, 0, -0.5, 1].*...
        [0; g; pitch*swingPitch/r];
    
    if ~contacts(1)
        f = swing1;
        f(2, :) = f(2, :) + [0, 1, 2, 1, 0]*fmin;
    elseif ~contacts(2)
        f = swing2;
        f(2, :) = f(2, :) + [1, 0, 1, 2, 0]*fmin;
    elseif ~contacts(3)
        f = stance;
        f(2, :) = f(2, :) + [2, 1, 0, 1, 0]*fmin;
    elseif ~contacts(4)
        f = stance;
        f(2, :) = f(2, :) + [1, 2, 1, 0, 0]*fmin;
    else
        f = stance;
        f(2, :) = f(2, :) + [1, 1, 1, 1, 0]*fmin;
    end
end

% Specify which grippers are currently engaged
function contacts = gait(t, k)
    contacts = ones(4, 1);
    if t > 2 & t < 4
        contacts(1) = 0;
    end
end

% Visualize the simulation results
function frames = animate(T, X, F, k)
    frames = struct('cdata',{},'colormap',{});
    
    % Compute desired viewing area
    xbounds = vrrotvec2mat([1, 0, 0, pi/2]) * reshape(X([1:3, 8:22], :), 3, []);
    bounds = [min(xbounds, [], 2) - 0.2, ...
              max(xbounds, [], 2) + 0.2];
    
    % Animate frames
    fig = figure('name', 'Microspine Climber');
    tic();
    for i = 1:length(T)
        % Terminate early if figure is closed
        if ~ishghandle(fig)
            return
        end
        % Draw frame if elapsed sim time > elapsed real time
        if toc() <= T(i) / k.ff || i == length(T)
            clf;
            
            % Plot current state
            drawrobot(X(:, i), k, T(i));
            drawforces(X(:, i), F(:, i), k);
            
            % Set up figure window
            axis equal;
            xlim(bounds(1, :));
            ylim(bounds(2, :));
            zlim(bounds(3, :));
            xlabel('x');
            ylabel('-z');
            zlabel('y');
            title(['', num2str(T(i), '%.3f'), ' Seconds']);
            
            % Display figure
            drawnow();
            if isempty(frames)
                frames = [getframe(gcf)];
            else
                frames = [frames, getframe(gcf)];
            end
        end
    end
end

% Draw the robot
function drawrobot(x, k, t)
    [p, q, r, ~, ~] = unpack(x);
    Rplot = vrrotvec2mat([1, 0, 0, pi/2]);
    R = q2R(q);
    c = [-1 1 -1 1 0; 1 1 -1 -1 -1; 0 0 0 0 0].*[k.w/2; k.l/2; k.h/2];
    face1 = p + R*(c(:, [1 2 4 3]) + [0; 0; k.h/2]);
    face2 = p + R*(c(:, [1 2 4 3]) - [0; 0; k.h/2]);
    c = p + R*c;
    
    c = Rplot*c;
    r = Rplot*r;
    plot3(r(1,:), r(2,:), r(3,:), 'b.', 'markersize', 50);
    hold on;
    if t > 2 & t < 4
        plot3(r(1,1), r(2,1), r(3,1), 'r.', 'markersize', 50);
    end
    for i = 1:5
        plot3([r(1, i), c(1, i)], [r(2, i), c(2, i)], ...
            [r(3, i), c(3, i)], 'k', 'linewidth', 5);
    end
    prism(Rplot*face1, Rplot*face2, 'b');
%     fill3(c(1,:), c(2,:), c(3,:), 'b');
end

% Draw the force vectors
function drawforces(x, f, k)
    [~, ~, r, ~, ~] = unpack(x);
    f = reshape(f, 3, []);
    Rplot = vrrotvec2mat([1, 0, 0, pi/2]);
    r = Rplot*r;
    f = Rplot*f/50;
    quiver3(r(1,:), r(2,:), r(3,:), f(1,:), f(2,:), f(3,:), 0, 'r');
end

% Plot the desired and actual forces over time
function plotforces(T, F, Fgoal)
    titles = {'Lateral Force', 'Tangential Force', 'Normal Force'};
    for i = 1:3
        figure('name', titles{i});
        hold on;
        labels = {'FL', 'FR', 'RL', 'RR', 'Tail'};
        ax = gca;
        colors = ax.ColorOrder;
        for j = 1:5
            plot(T, F(j*3+i-3, :), 'Color', colors(j, :), ...
                'displayname', labels{j});
            plot(T, Fgoal(j*3+i-3, :), '--', 'Color', colors(j, :), ...
                'HandleVisibility','off');
        end
        title(titles{i});
        xlabel('Time (s)');
        ylabel('Force (N)');
        legend
    end
end

% Split the state vector into components
function [p, q, r, v, w] = unpack(x)
    p = x(1:3, :);                          % Body position (3 x 1)
    q = x(4:7, :);                          % Body attitude (quaternion)
    r = reshape(x(8:22, :), 3, 5, []);      % Feet/tail positions (3 x 5)
    v = x(23:25, :);                        % Body velocity (3 x 1)
    w = x(26:28, :);                        % Body angular velocity (3 x 1)
end

% Rotation matrix from body to world frame
function R = q2R(q)
    R = quat2rotm(q');
end

% Quaternion left multiplication matrix
function L = q2L(q)
    s = q(1);
    v = q(2:4);
    L = [s -v'; v s*eye(3)+skew(v)];
end

% Grasp matrix
function G = grasp(r)
    G = zeros(6, 15);
    for i = 1:5
        G(1:3, i*3-2:i*3) = eye(3);
        G(4:6, i*3-2:i*3) = skew(r(:, i));
    end
end

% Skew symmetric matrix
function vhat = skew(v)
    vhat = [0, -v(3), v(2); v(3) 0 -v(1); -v(2) v(1) 0];
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

function cylinder(center1, center2, radius, varargin)
    x = linspace(0, 360, 20);
    R = vrrotvec2mat(vrrotvec([0; 0; 1], center2 - center1));
    face1 = R*radius*[cosd(x); sind(x); zeros(size(x))] + center1;
    face2 = face1 + center2 - center1;
    prism(face1, face2, varargin{:});
end

% for w = 0:.1:1
% % w = 0.2;
% rng(1);
% G = rand(6, 12);
% dx = rand(12,1);
% 
% G2 = G(:, 4:end);
% dx2 = dx(4:end);
% A2 = (eye(9) - G2'*inv(G2*G2')*G2);
% dy2 = A2*dx2;
% 
% I = eye(12);
% W = [w,w,w,1,1,1,1,1,1,1,1,1];
% % I(1:3, 1:3) = w;
% % G(:,1:3) = G(:,1:3)*w;
% I = I.*W;
% G = G.*W;
% A = (I - G'*inv(G*G')*G)
% dy = A*dx;
% 
% dy(4:end) - dy2;
% 
% plot(w, dy(1), 'r.');
% hold on
% plot(w, dy(2), 'b.');
% plot(w, dy(3), 'g.');
% plot(w, dy(4), 'rs');
% plot(w, dy(5), 'bs');
% plot(w, dy(6), 'gs');
% plot(w, dy(7), 'r*');
% plot(w, dy(8), 'b*');
% plot(w, dy(9), 'g*');
% plot(w, dy(10), 'ro');
% plot(w, dy(11), 'bo');
% plot(w, dy(12), 'go');
% end