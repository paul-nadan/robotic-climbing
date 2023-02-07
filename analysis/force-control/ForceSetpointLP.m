%% LP comparison
r = [-1 1 -1 1 0; 1 1 -1 -1 -3; -1 -1 -1 -1 -1].*[1;1;1]; % foot positions
yaw = [1; -1; 1; -1]; % gripper yaw angles
F1 = solveLP(r, 0*yaw, [0; 1; 0; 0; 0; 0], [1 1 1 1], 15, 45, 0, .5)

%% Inward force QP analysis
stance = [0 1 1 1];
k = 0:1:45;
margins = zeros(size(k));
ratios = zeros(size(k));
for i = 1:length(k)
[F, margin, ratio] = solveLP(r, k(i)*yaw, [0; 1; 0; 0; 0; 0], stance, 20, 45, 0, 0.5);
margins(i) = margin;
ratios(i) = ratio;
end
close all
plot(k, ratios);
xlabel('Gripper Angle Offset');
ylabel('Adhesion Ratio');
title('Stance');
ylim([0, 0.5]);
xlim([0, 45]);
figure
plot(k, margins);
xlabel('Gripper Angle Offset');
ylabel('Adhesion Margin');
title('Stance');
ylim([0, 1]);
xlim([0, 45]);

%% Derivation
syms N1 N2 N3 N4 N5 L Fx
eq1 = N2 + N3 + N4 + N5 == Fx;
eq2 = N3 == N2 + N4;
eq3 = -N2 + N3 + N4 + L*N5 == 1;
s1 = solve(eq1, eq2, eq3);

eq1 = N1 + N2 + N4 + N5 == Fx;
eq2 = N1 == N2 + N4;
eq3 = -N1 - N2 + N4 + L*N5 == 1;
s3 = solve(eq1, eq2, eq3);

%% Numerical

L = 3;

% 4 foot stance
N5 = 0:.001:0.6;
N12 = .5*(L-1)*N5-.5;
N34 = -0.5*(L+1)*N5+.5;
cost = max(-N12, 0)+max(-N34, 0);
optimum = 1/(L+1); % N34 = 0
ratio = min(cost)

set(0,'defaultfigureposition',[400 100 900 750])
close all;
plot(N5, N12/2, 'DisplayName', 'FL')
hold on
plot(N5, N12/2, '--', 'DisplayName', 'FR')
plot(N5, N34/2, 'DisplayName', 'RL')
plot(N5, N34/2, '--', 'DisplayName', 'RR')
plot(N5, cost, 'DisplayName', 'Ratio')
title('4 Leg Stance');
xlabel('Tail Normal Force');
ylabel('Foot Normal Force');
ylim([-1, 1]);
legend

% Front foot swing
N2 = L*N5/2 - N5/2 - 0.5;
N3 = -N5/2;
N4 = 1/2 - L*N5/2;
cost = max(-N2, 0)+max(-N4, 0);
optimum = 1/L;
ratio = min(cost)/0.5

% Engagement
cost2 = max(max(-N3, 0)*2, max(-N2, 0)+max(-N4, 0));
ratio_engage = min(cost2)/0.5

figure;
plot(N5, N5*0, 'DisplayName', 'FL');
hold on;
plot(N5, N2, 'DisplayName', 'FR');
plot(N5, N3, 'DisplayName', 'RL');
plot(N5, N4, 'DisplayName', 'RR');
plot(N5, cost/.5, 'DisplayName', 'Swing Ratio');
plot(N5, cost2/.5, 'DisplayName', 'Engage Ratio');
title('Front Foot Swing')
legend
xlabel('Tail Normal Force');
ylabel('Foot Normal Force');
ylim([-1, 1]);

% Rear foot swing
N1 = -N5/2;
N2 = L*N5/2 - 1/2;
N4 = 1/2 - L*N5/2 - N5/2;
cost = max(-N2, 0)+max(-N4, 0);
optimum = 1/(L+1); % N4 = 0
ratio = min(cost)/0.5

% Engagement
cost2 = max(max(-N1, 0)*2, max(-N2, 0)+max(-N4, 0));
ratio_engage = min(cost2)/0.5

figure;
plot(N5, N1, 'DisplayName', 'FL');
hold on;
plot(N5, N2, 'DisplayName', 'FR');
plot(N5, N5*0, 'DisplayName', 'RL');
plot(N5, N4, 'DisplayName', 'RR');
plot(N5, cost/.5, 'DisplayName', 'Swing Ratio');
plot(N5, cost2/.5, 'DisplayName', 'Engage Ratio');
title('Rear Foot Swing')
legend
xlabel('Tail Normal Force');
ylabel('Foot Normal Force');
ylim([-1, 1]);


%% Tail Optimization

L = 1:.01:5;
% Stance
N5 = 1./(L+1);
N12 = .5*(L-1).*N5-.5;
N34 = -0.5*(L+1).*N5+.5;
cost = max(-N12, 0)+max(-N34, 0);
ratio1 = cost;

% Front swing
N5 = 1./L;
N2 = L.*N5/2 - N5/2 - 0.5;
N3 = -N5/2;
N4 = 1/2 - L.*N5/2;
cost = max(-N2, 0)+max(-N4, 0);
ratio2 = cost/0.5;

% ratio2 = 1./L = 1./(2*tail + 1);

% Rear swing
N5 = 1./(L+1);
N1 = -N5/2;
N2 = L.*N5/2 - 1/2;
N4 = 1/2 - L.*N5/2 - N5/2;
cost = max(-N2, 0)+max(-N4, 0);
ratio3 = cost./0.5;

% Plot
tail = (L-1)/2;
figure
plot(tail, ratio1);
hold on;
plot(tail, ratio2);
plot(tail, ratio3, '--');
actual = 350/283.4;
plot([actual, actual], [0, 1], '--');
legend('Stance', 'Front Swing', 'Back Swing', 'Current Design');
xlabel('Tail Length (body lengths)');
ylabel('Max Force Ratio (normalized)');
title('Effect of Tail on Microspine Adhesion');

function [F, margin, ratio] = solveLP(r, yaw, wrench, stance, phi, theta, fmin, fmax)
    % x = [F1 F2 F3 F4 N5+ u] where Fi = [Bi Ti Ni+ Ni-]
    f = [zeros(17, 1); 1];
    Aeq = zeros(6, 18);
    beq = wrench;
    A = zeros(12, 18);
    H = zeros(18, 18);
    b = zeros(12, 1);
    lb = zeros(18, 1)-inf;
    ub = zeros(18, 1)+inf;
    for i = 1:4
        % Force/torque balance
        R = [cosd(yaw(i)) -sind(yaw(i)) 0;
             sind(yaw(i)) cosd(yaw(i)) 0;
             0 0 1];
        Aeq(1:3, i*4-3:i*4) = R*[1 0 0 0; 0 1 0 0; 0 0 1 -1];
        Aeq(4:6, i*4-3:i*4) = skew(r(:, i))*Aeq(1:3, i*4-3:i*4);
        
        % Normal force limit
        A(i*3-2, i*4-3:i*4) = [0 -tand(phi) 0 1]*stance(i);  % N- - T tan(theta)
        A(i*3-2, end) = -1*stance(i);  % -u
        
        % Lateral force limit
        A(i*3-1, i*4-3:i*4) = [1 -tand(theta) 0 0];  % B - T tan(phi)
        A(i*3, i*4-3:i*4) = [-1 -tand(theta) 0 0];  % -B - T tan(phi)
        
        % Bounds
        lb(i*4-3:i*4) = [-fmax, fmin, 0, 0]*stance(i);
        ub(i*4-3:i*4) = [fmax, fmax, fmax, fmax]*stance(i);
    end
    % Tail
    Aeq(1:3, 17) = [0; 0; 1];
    Aeq(4:6, 17) = skew(r(:, 5))*Aeq(1:3, 17);
    lb(17) = 0;
    ub(17) = fmax;
%     H = ones(size(H))*1;
%     [X,FVAL,EXITFLAG] = linprog(f,A,b,Aeq,beq,lb,ub);
    [X,FVAL,EXITFLAG] = quadprog(H,f,A,b,Aeq,beq,lb,ub);
    F = zeros(3, 5);
    F0 = zeros(3, 5);
    for i = 1:4
        R = [cosd(yaw(i)) -sind(yaw(i)) 0;
             sind(yaw(i)) cosd(yaw(i)) 0;
             0 0 1];
        F0(:, i) = [1 0 0 0; 0 1 0 0; 0 0 1 -1]*X(i*4-3:i*4);
        F(:, i) = R*[1 0 0 0; 0 1 0 0; 0 0 1 -1]*X(i*4-3:i*4);
    end
    F0(:, end) = [0; 0; 1]*X(17)
    F(:, end) = [0; 0; 1]*X(17);
    margin = -FVAL / tand(phi)
    ratio = max(-F0(3, :)./F0(2, :))
end
    
% Skew symmetric matrix
function vhat = skew(v)
    vhat = [0, -v(3), v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end