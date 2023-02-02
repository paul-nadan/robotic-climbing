% X is gripper anchor, Y is wrist anchor, x-y-z = right-front-up
% Origin is at pivot point
% Parameter vector: [gripper anchor (xyz), wrist anchor (xyz), 
%                    gripper anchor (xyz), wrist anchor (xyz),
%                    gripper anchor (xyz), wrist anchor (xyz)]
% Angles: pitch (x), yaw (y), roll (z)
global Lmax Lmin
Lmax = -Inf;
Lmin = Inf;
N = 2;
s.fmin = 0.11 * 4.4482216; % lb to N
s.rate = 0.5 * 175.126835; % lb/in to N/m
s.size = 0.05/sqrt(2); % m
s.size = 1;

options = optimoptions(@fmincon,'MaxFunctionEvaluations', 1e7,...
                                'Algorithm', 'interior-point',...
                                'Display', 'off',...
                                'CheckGradients', false,...
                                'SpecifyObjectiveGradient',false);

x0 = rand(6*N, 1)-.5;
x0 = [-1; 0; -1; -1; 1; 0];
x0 = [-1; 0; -1; -1; 1; 0; -1; 0; 1; -1; 1; 0];
% x0 = [-1; 0+sind(35); -1; -1; 1; 0; -1; 0-sind(35); 1; -1; 1; 0];
x0 = [-40/sqrt(2); -38+35; 40/sqrt(2); 
      -40/sqrt(2); 35; 0;
      -40/sqrt(2); -40/sqrt(2)+35; -40/sqrt(2);
      -40/sqrt(2); 35; 0]/1000;

plotTorqueGlobal(x0, s, 0);
Lmax
Lmin
plotGripper(x0*s.size);
% plotGripperAtState(x0, [pi/3, 0, 0]')
[Ta, Tb] = gripperTorque(x0)
eig(Ta)
eig(Tb)
return
lb = x0*0 - 2;
ub = x0*0 + 2;
for i = 1:N
    lb(6*(i-1)+2) = 0;
    lb(6*(i-1)+5) = 1;
    ub(6*(i-1)+2) = 0;
    ub(6*(i-1)+5) = 1;
    ub(6*(i-1)+6) = 0; % Springs can't cross
end

[x,fval,flag,~] = fmincon(@getCost, x0, [],[],[],[], lb,ub, ...
    @(x)getConstraints(x,s), options)
[x,fval,flag,~] = fmincon(@getCostGlobal, x0, [],[],[],[], lb,ub, ...
    @(x)getConstraints(x,s), options)
[Ta, Tb] = gripperTorque(x)
% plotTorqueGlobal(x, s);
legend('Original Pitch', 'Original Yaw', 'Original Roll', 'Optimal Pitch', 'Optimal Yaw', 'Optimal Roll');
eig(Ta)
eig(Tb)
plotGripper(x)

function [c, g] = getCost(x)
    c = x'*x/2;
    g = x;
%     c = x(6) + x(12);
%     c = sum(abs(x)); % L-1 norm, turn off gradient
end

function c = getCostGlobal(x)
    angles = -pi/2:pi/36:pi/2;
    T = zeros(3, 3, length(angles));

    for i = 1:length(angles)
        T(1,:,i) = gripperTorqueAtState(x, [angles(i);0;0]);
        T(2,:,i) = gripperTorqueAtState(x, [0;angles(i);0]);
        T(3,:,i) = gripperTorqueAtState(x, [0;0;angles(i)]);
    end
    T(isnan(T)) = 0;
    c = sum(T.^2, 'all');
end

function plotTorqueGlobal(x, s, clearPlot)
    angles = -pi/2:pi/360:pi/2;
    T = zeros(3, 3, length(angles));

    for i = 1:length(angles)
        T(1,:,i) = gripperTorqueAtState(x, [angles(i);0;0], s);
        T(2,:,i) = gripperTorqueAtState(x, [0;angles(i);0], s);
        T(3,:,i) = gripperTorqueAtState(x, [0;0;angles(i)], s);
    end
    T(isnan(T)) = 0;
    figure(2);
    if nargin > 2
        if clearPlot
            clf;
        end
    end
    plot(rad2deg(angles), reshape(T(1,1,:), 1, []));
    hold on;
    plot(rad2deg(angles), reshape(T(2,2,:), 1, []));
    plot(rad2deg(angles), reshape(T(3,3,:), 1, []), '--');
    legend('Pitch', 'Yaw', 'Roll')
    xlabel('Angle (deg)');
    ylabel('Torque (Nm)');
end

function [c, ceq] = getConstraints(x, s)
    [Ta, Tb] = gripperTorque(x);
    eigs = [NaN,1,1,1,1,1];
    ceq = [Ta; Tb] + [diag(eigs(1:3)); diag(eigs(4:6))];
    ceq = reshape(ceq(~isnan(ceq)), 1, []);
    c = [];
end

function [X, Y] = getParams(x)
    symmetric = 2;
    N = length(x)/6;
    X = cell(N,1);
    Y = cell(N,1);
    for i = 1:N
        X{i} = x(6*(i-1)+1:6*(i-1)+3);
        Y{i} = x(6*(i-1)+4:6*(i-1)+6);
    end
    if symmetric == 4 % Horizontal and vertical
        X = [X; cell(3,1)];
        Y = [Y; cell(3,1)];
        X{2} = X{1}.*[-1;1;1];
        Y{2} = Y{1}.*[-1;1;1];
        X{3} = X{1}.*[-1;1;-1];
        Y{3} = Y{1}.*[-1;1;-1];
        X{4} = X{1}.*[1;1;-1];
        Y{4} = Y{1}.*[1;1;-1];
    end
    if symmetric == 2
        X = [X; cell(N,1)];
        Y = [Y; cell(N,1)];
        for i = 1:N
            X{i+N} = X{i}.*[-1;1;1];
            Y{i+N} = Y{i}.*[-1;1;1];
        end
%         X{2}(1) = 0;
%         Y{2}(1) = 0;
    end
    if symmetric == 3
        X = [X; cell(2,1)];
        Y = [Y; cell(2,1)];
        R = vrrotvec2mat([0, 1, 0, 2*pi/3]);
        X{2} = R*X{1};
        Y{2} = R*Y{1};
        X{3} = R*X{2};
        Y{3} = R*Y{2};
    end
end

function [Ta, Tb] = gripperTorque(x)
    [X, Y] = getParams(x);
    Ta = zeros(3,3);
    Tb = zeros(3,3);
    for i = 1:length(X)
        [Tai, Tbi] = springTorqueNL(X{i}, Y{i});
        Ta = Ta + Tai;
        Tb = Tb + Tbi;
    end
end

function T = gripperTorqueAtState(x, theta, s)
    if nargin < 3
        s.fmin = 1;
        s.rate = 1;
        s.size = 1;
    end
    x = x*s.size;
    [X, Y] = getParams(x);
    T = zeros(3,1);
    for i = 1:length(X)
        T = T + springTorqueAtState(X{i}, Y{i}, theta, s);
    end
end

function plotGripper(x)
    [X, Y] = getParams(x);
    figure(1);
    clf;
    for i = 1:length(X)
        plotLine(X{i}, Y{i}, 'k');
        hold on;
    end
    plot3(0, 0, 0, 'k.', 'MarkerSize', 20);
    O = [0;0;0];
    fillPolyhedron([X; {O}], 'r');
    fillPolyhedron([Y; {O}], 'b');
    xlabel('x'); ylabel('y'); zlabel('z');
    axis equal;
end

function plotGripperAtState(x, theta)
    [X, Y] = getParams(x);
    R = vrrotvec2mat([theta'/norm(theta), (norm(theta))]);
    figure(1);
    clf;
    for i = 1:length(X)
        Y{i} = R*Y{i};
        plotLine(X{i}, Y{i}, 'k');
        hold on;
    end
    plot3(0, 0, 0, 'k.', 'MarkerSize', 20);
    O = [0;0;0];
    fillPolyhedron([X; {O}], 'r');
    fillPolyhedron([Y; {O}], 'b');
    xlabel('x'); ylabel('y'); zlabel('z');
    axis equal;
end

function fillPolygon(X, c)
    x = zeros(size(X));
    y = zeros(size(X));
    z = zeros(size(X));
    for i = 1:length(X)
        x(i) = X{i}(1);
        y(i) = X{i}(2);
        z(i) = X{i}(3);
    end
    fill3(x, y, z, c);
end

function fillPolyhedron(X, c)
    for i = 1:length(X)-2
        for j = i:length(X)-1
            for k = j:length(X)
                fillFace(X{i}, X{j}, X{k}, c)
            end
        end
    end
end

function plotLine(A, B, c)
    plot3([A(1), B(1)], [A(2), B(2)], [A(3), B(3)], c, 'LineWidth', 4);
end

function fillFace(A, B, C, c)
    fill3([A(1), B(1), C(1)], [A(2), B(2), C(2)], [A(3), B(3), C(3)], c);
end

function fillTetrahedron(A, B, C, D, c)
    fillFace(A, B, C, c);
    fillFace(A, B, D, c);
    fillFace(A, D, C, c);
    fillFace(D, B, C, c);
end

% syms x1 y1 z1 x2 y2 z2 L real
% wx = [1; 0; 0];
% wy = [0; 1; 0];
% wz = [0; 0; 1];
% 
% X1 = [x1; y1; z1];
% X2 = [y1; y2; z2];
% 
% % X1 = [0; 1; 1];
% % X2 = [1; -1; 0];
% 
% dx = X2 - X1;
% % L = norm(dx)
% 
% Tau = [(cross(X2, dx/L) * dot(cross(wx, X2), dx/L))';
%        (cross(X2, dx/L) * dot(cross(wy, X2), dx/L))';
%        (cross(X2, dx/L) * dot(cross(wz, X2), dx/L))'];
% Tau = simplify(Tau)
%    eig(Tau)
% 
% % taux = simplify(cross(X2, dx/L) * dot(cross(wx, X2), dx/L))
% % tauy = simplify(cross(X2, dx/L) * dot(cross(wy, X2), dx/L))
% % tauz = simplify(cross(X2, dx/L) * dot(cross(wz, X2), dx/L))
% 
% L = 1;
% S = skew(X1).*[1;1;-1]
% S*X2
% % S2 = [-1;1;1].*skew(X1.*[-1;1;1]);
% % 
% % S = S1+S2;
% 
% A = [S(1,:) - S(3,:); S(1,:)]
% b = [0;1];
% X = A\b
% S*X
% return
% 
% dx = X2 - X1;
% taux = (cross(X2, dx/L) * dot(cross(wx, X2), dx/L))
% tauy = (cross(X2, dx/L) * dot(cross(wy, X2), dx/L))
% tauz = (cross(X2, dx/L) * dot(cross(wz, X2), dx/L))

function A = skew(x)
    A = [0, -x(3), x(2);
         x(3), 0, -x(1);
         -x(2), x(1), 0];
end

function T = springTorque(X, Y, L)
    if nargin < 3
        L = norm(X-Y);
    end
    dx = Y - X;
    wx = [1; 0; 0];
    wy = [0; 1; 0];
    wz = [0; 0; 1];
    T = -[(cross(Y, dx/L) * dot(cross(wx, Y), dx/L))';
          (cross(Y, dx/L) * dot(cross(wy, Y), dx/L))';
          (cross(Y, dx/L) * dot(cross(wz, Y), dx/L))'];
end

function [T1, T2] = springTorqueNL(X, Y, L)
    if nargin < 3
        L = norm(X-Y);
    end
    dx = Y - X;
    wx = [1; 0; 0];
    wy = [0; 1; 0];
    wz = [0; 0; 1];
    T1 = -[(cross(Y, dx/L) * max(0, dot(cross(wx, Y), dx/L)))';
          (cross(Y, dx/L) * max(0, dot(cross(wy, Y), dx/L)))';
          (cross(Y, dx/L) * max(0, dot(cross(wz, Y), dx/L)))'];
    T2 = -[(cross(Y, dx/L) * min(0, dot(cross(wx, Y), dx/L)))';
          (cross(Y, dx/L) * min(0, dot(cross(wy, Y), dx/L)))';
          (cross(Y, dx/L) * min(0, dot(cross(wz, Y), dx/L)))'];

end

function T = springTorqueAtState(X0, Y0, theta, s)
    global Lmax Lmin
    L0 = norm(X0-Y0);
    R = vrrotvec2mat([theta'/norm(theta), (norm(theta))]);
    Y = R*Y0;
    X = X0;
    dx = Y - X;
    L = norm(dx);
    if L > Lmax
        Lmax = L-L0;
    end
    if L < Lmin
        Lmin = L-L0;
    end
    dL = max(0,L-L0);
    f = dL*s.rate + sign(dL)*s.fmin;
    F = f*dx/L;
    T = -cross(Y, F);
end