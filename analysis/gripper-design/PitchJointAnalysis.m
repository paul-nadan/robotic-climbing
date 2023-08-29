h = 2;
px = 0.25;
py = 0.5;

close all;
set(0,'defaultfigureposition',[400 100 900 750])

% plot_radius(px, py, h)
% plot_radius_heatmap()
% sym_contact_forces()
% plot_contact_forces(-90, px, py, 1, 2)
% plot_contact_forces_heatmap(px, py);
plot_contact_forces_heatmap_flat();
% plot_contact_forces_heatmap_xy();

%% Gripper load angle functions

% Compute normalized forces at equilbrium under external load
% Forces are in world frame
% Load angle is CCW from +x
function [F1x, F1y, F2x, F2y, F3x, F3y, offset, a_contact, a_load] = ...
        contact_forces(load_angle, px, py, a1, a2)
    
    s = size((load_angle + px + py + a1 + a2));
    load_angle = load_angle + zeros(s);
    px = px + zeros(s);
    py = py + zeros(s);
    a1 = a1 + zeros(s);
    a2 = a2 + zeros(s);
    
    N1x = cosd(a1);
    N1y = sind(a1);
    T1x = -sind(a1);
    T2x = cosd(a1);
    N2x = cosd(a2);
    N2y = sind(a2);
    F3x = cosd(load_angle);
    F3y = sind(load_angle);
    tau = .2;
    
    F1x = F3y.*px + tau - F3x.*py;
    F1y = (F3x.*N2y - F3y.*N2x + F3y.*N2y.*px - F3x.*N2y.*py + N2y.*tau)./N2x;
    f2 = -(F3x + tau + F3y.*px - F3x.*py)./N2x;
    
    o = f2 <= 0;
    F1x(o) = -F3x(o);
    F1y(o) = -F3y(o);
    f2(o) = 0;
    offset = zeros(size(f2));
    offset(o) = atan2d(1-py(o), px(o)) + load_angle(o);
    offset(offset < 0) = 100; % Placeholder: solve with torque
    
    F2x = f2.*N2x;
    F2y = f2.*N2y;
    
    a_contact = offset - a1;
    a_load = mod(atan2d(F1y, F1x) - a1 + 90, 360) - 180;
end

function sym_contact_forces()
    syms F1x F1y f2 F3x F3y N2x N2y p_x p_y tau real
    F1 = [F1x; F1y];
    F2 = f2*[N2x; N2y];
    F3 = [F3x; F3y];
    p = [p_x; p_y];
    eq1 = F1 + F2 + F3 == 0;
    eq2 = cross([p;0], [F3;0]) == -cross([0;1;0], [F1;0]) - [0; 0; tau];
    s = solve([eq1; eq2(3)], [F1x, F1y, f2]);
    s.F1x
    s.F1y
    s.f2
end

function plot_contact_forces_heatmap(px, py)
    k = linspace(0, 2);
    a = -180:0;
    [K, A] = meshgrid(k, a);
    r = 1./K;
    dx = -sqrt(r.^2 - 1/4);
    a1 = atan2d(1/2, -dx);
    a2 = -a1;
    [F1x, F1y, F2x, F2y, F3x, F3y, offset, a_contact, a_load] = ...
        contact_forces(A, px, py, a1, a2);
    r_min = min_radius(px, py);
    
    imagesc(k, a, a_load);
    axis('xy');
    hold on;
    contour(K, A, a_contact, [10 10], 'r--', 'linewidth', 5);
    contour(K, A, a_load - min(0, a_contact), [20 20], 'm--', 'linewidth', 5);
%     contour(K, A, offset, [1e-3 1e-3], 'b--', 'linewidth', 5);
    contour(K, A, K-1./r_min, [1e-3 1e-3], 'g--', 'linewidth', 5);
    xlabel('Curvature');
    ylabel('Gripper Loading Angle');
    title('Spine Loading Angle');
    legend('Contact Limit', 'Load Limit', 'Collision Limit', 'location', 'southeast');
    colorbar;
end

function plot_contact_forces_heatmap_flat()
    px = linspace(0, 1, 500);
    py = 0.5;
    a = -180:.2:0;
    [P, A] = meshgrid(px, a);
    [F1x, F1y, F2x, F2y, F3x, F3y, offset, a_contact, a_load] = ...
        contact_forces(A, P, py, 0, 0);    
    A = A + 90;
    a = a + 90;
    imagesc(px, a, a_load);
    axis('xy');
    hold on;
    contour(P, A, a_contact, [1e-3 1e-3], 'r', 'linewidth', 5);
    contour(P, A, a_load - min(0,a_contact), [20 20], 'b', 'linewidth', 5);
%     contour(P, A, offset, [1e-3 1e-3], 'm--', 'linewidth', 5);
    xlabel('Pivot X');
    ylabel('Gripper Loading Angle');
    title('Spine Loading Angle');
    legend('Contact Angle Limit', 'Load Angle Limit', 'Offset', 'location', 'southeast');
    colorbar;
end

function plot_contact_forces_heatmap_xy()
    n = 500; % 500 for high resolution
    px = linspace(0, 1, n);
    py = linspace(0, 1, n);
    A = flip(-180:.1:0);
    [Px, Py] = meshgrid(px, py);
    Amax = A(1)*ones(size(Px))+90;
    for a = A
        [F1x, F1y, F2x, F2y, F3x, F3y, offset, a_contact, a_load] = ...
            contact_forces(a, Px, Py, 0, 45);
        A1 = a_contact <= 0;
        A2 = a_load - min(0,a_contact) <= 20;
        Amax(~(A1 & A2)) = a + 90;
    end
    imagesc(px, py, Amax);
    caxis([-90, 20]);
    axis('xy');
    axis equal;
    xlim([min(px), max(px)]);
    ylim([min(py), max(py)]);
    xlabel('Pivot X');
    ylabel('Pivot Y');
    title('Maximum Gripper Load Angle');
%     legend('Contact Angle Limit', 'Load Angle Limit', 'Offset', 'location', 'southeast');
    colorbar;
end

% Visualize contact forces
function plot_contact_forces(load_angle, px, py, h, r)
    if nargin < 5
        r = min_radius(px, py);
    end
    dx = -sqrt(r^2 - 1/4);
    t = linspace(0, 2*pi);
    a1 = atan2d(1/2, -dx);
    a2 = -a1 + 0;
    
    [F1x, F1y, F2x, F2y, F3x, F3y, offset] = contact_forces(load_angle, px, py, a1, a2);
    
    R = [cosd(offset), -sind(offset); sind(offset), cosd(offset)];
    F1 = [F1x; F1y];
    F2 = [F2x; F2y];
    F3 = [F3x; F3y];
    p = R*[px; py-1];
    c = R*[0; -1];
    quiver(0, 0, F1(1), F1(2), 'b', 'linewidth', 3);
    hold on;
    axis equal;
    quiver(c(1)*h, c(2)*h, F2(1), F2(2), 'b', 'linewidth', 3);
    quiver(p(1)*h, p(2)*h, F3(1), F3(2), 'b', 'linewidth', 3);
    
    plot(r*cos(t)*h+dx*h, r*sin(t)*h-h/2, 'm');
    T1 = [-sind(a1); cosd(a1)] .* [.2 -.2];
    T2 = [-sind(a2); cosd(a2)] .* [.2 -.2] + [0; -h];
    plot(T1(1,:), T1(2,:), 'k', 'linewidth', 5);
    plot(T2(1,:), T2(2,:), 'k', 'linewidth', 5);
%     plot([0, p(1)*h, c(1)*h], [0, p(2)*h, c(2)*h], 'k--', 'linewidth', 2);
    plot(p(1)*h, p(2)*h, 'r.', 'markersize', 40);
    plot(c(1)*h, c(2)*h, 'r.', 'markersize', 40);
    plot(0, 0, 'r.', 'markersize', 40);
    xlim([-1.5, 1.5]*h);
    ylim([-2, 1]*h);
end

%% Gripper curvature functions

% Compute radius of curvature given pivot location
% x is normalized pivot distance from front plane
% y is normalized pivot height from lower contact
% r is normalized minimum radius of curvature
function r = min_radius(px, py)
    r = 1/2 * sqrt((px.^2+py.^2).*(px.^2+py.^2-2.*py+1))./px;
    r(px.^2 + (py-.5).^2 > 1/4) = 1/2;
end

% Visualize maximum curvature over all possible pivot locations
function plot_radius_heatmap()
    px = linspace(0, 1);
    py = linspace(0, 1);
    [X, Y] = meshgrid(px, py);
    K = 1./(min_radius(X, Y));
    imagesc(px, py, K);
    hold on;
    t = linspace(-pi/2, pi/2);
%     plot(1/2*cos(t), 1/2*sin(t)+1/2, 'r--');
%     plot(0,0,'k.', 'markersize', 40);
%     plot(0,1,'k.', 'markersize', 40);
    xlabel('Pivot X');
    ylabel('Pivot Y');
    axis equal;
    xlim([0,1]);
    ylim([0,1]);
    title('Maximum Curvature');
    colorbar;
end

% Visualize maximum radius of curvature for specified gripper geometry
% h is distance between gripper contacts
% px*h is pivot distance from front plane
% py*h is pivot height from lower contact
function plot_radius(px, py, h)
    r = min_radius(px, py)*h;
    dx = -sqrt(r^2 - (h/2)^2);
    t = linspace(0, 2*pi);
    
    plot(px*h, py*h-h, 'r.', 'markersize', 40);
    hold on;
    axis equal;
    plot(0, -h, 'r.', 'markersize', 40);
    plot(0, 0, 'r.', 'markersize', 40);
    plot(r*cos(t)+dx, r*sin(t)-h/2, 'b');
end