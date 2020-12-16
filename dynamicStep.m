addpath('terrain-generation', 'config-generation', 'robot-kinematics', ...
    'optimization', 'visualization', 'dynamics');

config = quadruped([3,3,2,2], 0.1, 0.3, {.2, [.16, .16]}, 0, 2);
seed = 42;
grid = terrain([-.75, 1.25], ...
        [-1.5 0.5], .01, 0*[1,1,0.5], [1, .25, 0.0625], 0, [0;-.75;0], seed);
% robot1 = spawnRobot(grid.spawn, eye(3), config, grid);
% robot2 = step(robot1, 1, grid);
robot1 = getRobot([0;0;0], eye(3), config.gait.angles(:,1), config);
feet = find(config.gait.feet(:, 1));

[g_pl, g_sl, m_l, m_o] = getFrames(config); % Reference frames and masses
g_wo = [robot1.R0, robot1.origin; 0 0 0 1];
g_fc = getContactFrames(robot1, feet, g_pl, g_wo, grid); % Contact frames

% Setup
g = 9.81;
B_c = eye(6, 3);
f = length(feet); % Number of foot contacts
c_b = size(B_c, 2); % Number of constraints per foot
c = c_b*f; % Number of constraints total
n = length(g_pl); % Number of links
e = n + 6; % Number of state variables
syms q(t) [e, 1]
syms q_dot [e, 1] real
assume(q(t), 'real')
q = q(t);

% Mass Matrix
% Jb_wp = jacobianB(g_wo, q(end-5:end));
Jb_wp = eye(6);%[robot1.R0', zeros(3); zeros(3,6)];
M_11 = 0;
M_21 = 0;
M_12 = 0;
M_22 = 0;
for iLink = 1:n
    Ad = tform2adjoint(inv(g_pl{iLink}))*Jb_wp;
    J = jacobianB(g_pl{iLink}, q(1:end-6));
    M_11 = M_11 + simplify(J'*m_l{iLink}*J);
    M_21 = M_21 + simplify(Ad'*m_l{iLink}*J);
    M_12 = M_12 + simplify(J'*m_l{iLink}*Ad);
    M_22 = M_22 + simplify(Ad'*m_l{iLink}*Ad);
end
M_22 = M_22 + Jb_wp'*m_o*Jb_wp;
M_bar = simplify([M_11 M_12; M_21 M_22]);
fprintf('Computed Mass Matrix\n');
C_bar = zeros(size(M_bar));
% C_bar = coriolis(M_bar, q, q_dot);
fprintf('Computed Coriolis Matrix\n');
V_f = 0;
for iLink = 1:n
    g_wl = g_wo*g_pl{iLink};
    V_f = V_f + g_wl(4,3)*m_l{iLink}(1,1)*g;
end
V_o = q(n+2)*m_o(1,1)*g;
N_bar = gradientF(V_f*0 + V_o, q);
fprintf('Computed N Matrix\n');

% Grasp Constraints
syms J_h [c, n] real;
syms G_s [6, c] real;
for iFoot = 1:f
    g_sc = g_sl{feet(iFoot)}*g_fc{iFoot};
    Js_sf = jacobianS(g_sl{feet(iFoot)}, q(1:end-6));
    J_h(iFoot*c_b+1-c_b:iFoot*c_b, :) = B_c'*tform2adjoint(inv(g_sc))*Js_sf;
    g_oc = g_pl{feet(iFoot)}*g_fc{iFoot};
    G_s(:, iFoot*c_b+1-c_b:iFoot*c_b) = -tform2adjoint(inv(g_oc))'*B_c;
end
Jb_po = eye(6);
G_bar_s = Jb_po'*G_s;
A = simplify([-J_h, G_bar_s']);
A_dot = diff(A, t);
A_dot = simplify(subs(A_dot, diff(q), q_dot));
fprintf('Computed Constraints\n');

% Backward Solution
% q_ddot = zeros(e, 1);
% Y = M_bar*q_ddot + C_bar*q_dot + N_bar + A'*lambda;

% Forward Solution
% TODO: substitution
keys = [q; q_dot];
vals = [deg2rad(robot1.angles); robot1.origin; zeros(3+length(q), 1)];
M_bar_sub = double(subs(M_bar, keys, vals));
C_bar_sub = double(subs(C_bar, keys, vals));
N_bar_sub = double(subs(N_bar, keys, vals));
A_sub = double(subs(A, keys, vals));
A_dot_sub = double(subs(A_dot, keys, vals));
q_dot_sub = double(subs(q_dot, keys, vals));

fprintf('Beginning Forward Computation\n');
[F, Fnorm, Ftang, T] = quasiStaticDynamics(robot1, 0, grid);
T_axis = zeros(n, 1);
for iLink = 1:n
    T_axis(iLink) = T(:,iLink)'*config.joints(:,2,iLink);
end
Y = [T_axis; zeros(6, 1)];
state = [M_bar_sub, A_sub'; A_sub, zeros(c)]\[Y - C_bar_sub*q_dot_sub - N_bar_sub; -A_dot_sub*q_dot_sub];
q_ddot = state(1:e); % Acceleration
lambda = state(e+1:end); % Lagrange multipliers
fprintf('Completed Forward Computation\n');
dynamicForce = A_sub'*lambda
quasiForce = F
footForces = {};
footForces{1} = -A_sub(1:3,:)'*lambda(1:3);
footForces{1}(end-5:end-3)
footForces{2} = -A_sub(4:6,:)'*lambda(4:6);
footpiugfr1`Forces{2}(end-5:end-3)
footForces{3} = -A_sub(7:9,:)'*lambda(7:9);
footForces{3}(end-5:end-3)

plotTerrain(grid);
plotRobot(robot1);
% plotRobot(robot2);
for iFoot = 1:length(feet)
    g_wc = g_wo*g_pl{feet(iFoot)}*g_fc{iFoot};
    subbed = double(subs(g_wc, q, [deg2rad(robot1.angles); robot1.origin; 0;0;0]));
    p = subbed(1:3, 4);
    scale = 0.01;
    N = footForces{iFoot}(end-5:end-3);
    quiver3(p(1), -p(3), p(2), N(1)*scale, ...
            -N(3)*scale, N(2)*scale, 'r', 'linewidth', 2);
    N = quasiForce(:,iFoot);
    quiver3(p(1), -p(3), p(2), N(1)*scale, ...
            -N(3)*scale, N(2)*scale, 'g', 'linewidth', 2);
    plotPoints(p, 'g.');
end