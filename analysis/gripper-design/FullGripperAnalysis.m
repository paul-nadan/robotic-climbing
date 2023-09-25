close all
axial = true;
set(groot,'defaultLineLineWidth',4);
set(0, 'DefaultAxesLineWidth', 4);
set(groot,'defaultAxesFontSize',28);
lsize = 36; % Axis label font size
tsize = 41; % Title font size
lwidth = 2; % Legend border

syms w h theta x z real positive
syms f1t f1n f2t f2n fbz fx fy fz tx ty real

% Rotate spine forces into gripper frame
f1x = f1t * cos(theta);
f1y = f1t * sin(theta);
f1z = f1n;
f2x = f2t * cos(theta);
f2y = -f2t * sin(theta);
f2z = f2n;

% Assumptions
fbx = 0;    % No friction at base
fby = 0;    % No friction at base
tz = 0;     % No yaw moment
y = 0;      % Symmetry

% Force vectors
f1 = [f1x; f1y; f1z];
f2 = [f2x; f2y; f2z];
fb = [fbx; fby; fbz];
f = [fx; fy; fz];
t = [tx; ty; tz];

% Position vectors
r1 = [h; w/2; 0];
r2 = [h; -w/2; 0];
rb = [0; 0; 0];
r = [x; y; z];

% Static equilibrium
eq_force = f1 + f2 + fb + f == [0; 0; 0];
eq_torque = cross(r1, f1) + cross(r2, f2) + cross(rb, fb) + cross(r, f) ...
            + t == [0; 0; 0];
eq = [eq_force; eq_torque(1:2)]
eq = subs(eq, theta, pi/4);

% Contact force solutions
s = solve(eq, f1t, f1n, f2t, f2n, fbz);
s = [s.f1t; s.f1n; s.f2t; s.f2n; s.fbz];

% Only consider one axis at a time
sx = simplify(subs(s, [fy, tx, h], [0, 0, h]))
sy = simplify(subs(s, [ty, h], [0, 1]))

% Convert from force vector to angle
syms phi real
sx_ratio = simplify(-sx(2) / sx(1));
sx_ratio_subs = simplify(subs(sx_ratio, [ty, fz, fx], [0, sin(phi), -cos(phi)]))
sx_fbz_subs = simplify(subs(sx(end), [ty, fz, fx], [.2, sin(phi), -cos(phi)]))

sy_ratio1 = simplify(-sy(2) / sy(1));
sy_ratio2 = simplify(-sy(4) / sy(3));
sy_ratio_subs1 = simplify(subs(sy_ratio1, [tx, fy, fx], [0, sin(phi), -cos(phi)]))
sy_ratio_subs2 = simplify(subs(sy_ratio2, [tx, fy, fx], [0, sin(phi), -cos(phi)]))
sy_fbz_subs = simplify(subs(sy(end), [tx, fy, fx], [0, sin(phi), -cos(phi)]))

% Constraint functions
c1 = matlabFunction(sx_fbz_subs); % > 0
c2 = matlabFunction(tand(20) - sx_ratio_subs); % > 0

c3 = matlabFunction(sy_fbz_subs);
c4 = matlabFunction(tand(20) - sy_ratio_subs1); % > 0
c5 = matlabFunction(tand(20) - sy_ratio_subs2); % > 0

% Plot results
close all;
res = 400;
phi_vec = linspace(-pi/2, pi/2, res);
x_vec = linspace(0, 1, res);
z_vec = linspace(0, 1, res);
[X, Z, Phi] = meshgrid(x_vec, z_vec, phi_vec);

C1 = c1(1, Phi, X, Z);
C2 = c2(1, Phi, X, Z);
[val, ind] = max(min(C1, C2) < 0, [], 3);
ind(val == 0) = length(phi_vec);
PhiMax = phi_vec(ind);
imagesc(z_vec, x_vec, rad2deg(PhiMax)');
xlabel('Pivot Depth $(z)$');
ylabel('Pivot Height $(x)$');
cb = colorbar();
for i = 1:length(cb.TickLabels)
    cb.TickLabels{i} = [cb.TickLabels{i}, '^\circ']
end

title('Max Pull-Off Angle ($\phi_{max}$)');
set (gca, 'ydir', 'normal');
box off;
legend boxoff;

% phi_vec = linspace(0, pi/2, res);
% [X, Z, Phi] = meshgrid(x_vec, z_vec, phi_vec);
% % C3 = c3(Phi, Z);
% C4 = c4(-1, Phi, 1, X, Z);
% C5 = c5(-1, Phi, 1, X, Z);
% figure();
% [val, ind] = max(C4 < 0 | C5 < 0, [], 3);
% ind(val == 0) = length(phi_vec);
% PhiMax = phi_vec(ind);
% imagesc(z_vec, x_vec, rad2deg(PhiMax)');
% xlabel('Pivot $z$');
% ylabel('Pivot $x$ * Fz');
% colorbar;
% title('Maximum Loading Angle (deg) - Roll');
% set (gca, 'ydir', 'normal');