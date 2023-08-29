% x is into surface, y is up
% state is [outer pivot xy, outer angle, inner angle, spine tip xy]
% 1 is outer (gripper), 2 is inner (carriage)
% Neglect mass/inertia

k.F = [-1; -2];       % Applied force in N

k.kt = 10;              % Tangential stiffness in N/m
k.kn = 1;              % Normal stiffness in N/m
k.k = 100;              % Lower contact stiffness in N/m
k.d = 10;               % Outer pivot damping in Ns/m
k.k1 = 1;               % Torsional stiffness of outer pivot (Nm/deg)
k.k2 = 1;               % Torsional stiffness of inner pivot (Nm/deg)
k.tau1 = 0;             % Preload torque of outer pivot (Nm)
k.tau2 = 0;             % Preload torque of inner pivot (Nm)
k.pivot = [.1; 1*0];       % Inner pivot relative to outer pivot (m)
k.palm = [.5; -1.7 - .8*0];       % Outer lower contact relative to outer pivot (m)
k.spine = [1.4; 1.7];       % Spine tip relative to inner pivot (m)
k.carriage = [0.4; -1];   % Inner lower contact relative to inner pivot (m)
k.rigid = 1;            % Lock inner rotation (true/false)

k.playbackspeed = 50;    % Scale playback speed

x0 = [-k.pivot - k.spine; 0; 0; 0; 0; zeros(6, 1)];
tspan = [0, 100];

[T, X] = ode45(@(t, x)update(t, x, k), tspan, x0);
x = X(end,:)';
[~, Fspine] = update(T(end), x, k)
animateGripper(T, X, k)

function [dx, Fspine] = update(t, x, k)
    dx = zeros(12, 1);
    p1 = [x(1); x(2)];
    R1 = getR(x(3));
    R2 = getR(x(4));
    s = [x(5); x(6)];
    p2 = p1 + R1*k.pivot;
    palm = p1 + R1*k.palm;
    carriage = p2 + R1*R2*k.carriage;
    spine = p2 + R1*R2*k.spine;
    
    Fspine = [k.kn; k.kt] .* (s - spine);
    Fpalm = [k.k * -palm(1); 0];
    Fcarriage = [k.k * -carriage(1); 0];
    Fdamping = -k.d * x(7:8);
    
    % Normal force constraints
    Fpalm = min(0, Fpalm);          % Normal force
    Fcarriage = min(0, Fcarriage);  % Normal force
    
    % Kinematics
    dx(1:6) = x(7:12);
    
    % Forces/torques
    dx(7:8) = k.F + Fspine + Fpalm + Fcarriage + Fdamping;
    dx(9) = cross2(spine - p1, Fspine) + cross2(palm - p1, Fpalm) + cross2(carriage - p1, Fcarriage);
    dx(10) = cross2(spine - p2, Fspine) + cross2(carriage - p2, Fcarriage);
    dx(11:12) = -Fspine;
    
    % Spine constraints
    dx(11) = 0;
    dx(12) = max(0, dx(12)) - .1*x(12);
    
    if k.rigid
        dx(10) = 0;
    end
end

function R = getR(a)
    R = [cosd(a) -sind(a); sind(a) cosd(a)];
end

function w = cross2(u, v)
    w = u(1)*v(2) - v(1)*u(2);
end

function animateGripper(T, X, k)
    close all;
    set(0,'defaultfigureposition',[400 100 900 750]);
    fig = figure('name', 'Gripper');
    
    tic();
 
    for i = 1:length(T)
        % Terminate early if figure is closed
        if ~ishghandle(fig)
            return
        end
        % Draw frame if elapsed sim time > elapsed real time
        if toc() <= T(i) / k.playbackspeed || i == length(T)
            drawGripper(X(i, :), k);
            drawnow();
        end
    end
end

function drawGripper(x, k)
    p1 = [x(1); x(2)];
    R1 = getR(x(3));
    R2 = getR(x(4));
    s = [x(5); x(6)];
    p2 = p1 + R1*k.pivot;
    
    clf;
    drawLine([0, -5], [0, 0], 'k');
    hold on;
    axis equal;
    drawLine(p1, p2, 'r');
    drawLine(p1, p1 + R1*k.palm, 'r');
    drawLine(p2, p2 + R1*R2*k.spine, 'b');
    drawLine(p2, p2 + R1*R2*k.carriage, 'b');
    drawLine(p2 + R1*R2*k.spine, s, 'g');
    plot(p1(1), p1(2), 'k.', 'markersize', 100);
    plot(p2(1), p2(2), 'k.', 'markersize', 100);
end

function drawLine(p1, p2, c)
    plot([p1(1), p2(1)], [p1(2), p2(2)], c, 'linewidth', 20);
end