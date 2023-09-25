%% 
solveQP = @solveLP;
solveQP_old = @solveLP1;

% %% Timing code
% n = 100000;
% a = zeros(3, n);
% tic()
% for i = 1:n
%     Ji = getJ([rand(), rand(), rand()]');
%     a(:, i) = Ji(:,1);
% end
% toc()
% 
% tic()
% for i = 1:n    
%     Ji = getJoints(rand(3,4));
%     a(:, i) = Ji(:,1);
% end
% toc()
%% Force/adhesion during a step (actual geometry)
getJ = matlabFunction(J);
getX = matlabFunction(X);
r = [-1 1 -1 1 0; 1 1 -1 -1 -3; -1 -1 -1 -1 -1].*[0.154;0.283;0];
yaw = [1; -1; 1; -1];
g = 9.81*2;
h = 0.15;

z0 = -24.6442/1000;

angles = [repmat([0; -45; 30], 1, 4), [0; 0; -20]];

% F1 = solveLP1(r, 0*yaw, [0; g; 0; -g*h; 0; 0], [1 1 1 1], 20, 45, 0, 20)

F = solveLP(r, 30*yaw, [0; g; 0; -g*h; 0; 0], [0 1 1 1], 20, 45, 2, 20)
tau = torques(F, angles, getJ)

% F2 = solveLP(r, 0*yaw, [0; g; 0; -g*h; 0; 0], [0 1 1 1], 15, 45, 0, 20)
% F3 = solveLP(r, 0*yaw, [0; g; 0; -g*h; 0; 0], [1 1 0 1], 15, 45, 0, 20)


%% LP comparison
r = [-1 1 -1 1 0; 1 1 -1 -1 -3; -1 -1 -1 -1 -1].*[1;1;1]; % foot positions
yaw = [1; -1; 1; -1]; % gripper yaw angles
F1 = solveLP(r, 0*yaw, [0; 1; 0; 0; 0; 0], [1 1 1 1], 15, 45, 0, .5)

%% Inward force QP analysis
stance = [1 1 0 1];
getJ = matlabFunction(J);
g = 9.81*2;
h = 0.1;
fg = [0; g; 0; -g*h; 0; 0];
r = [-1 1 -1 1 0; 1 1 -1 -1 -3; -1 -1 -1 -1 -1].*[0.154;0.283;0];

% [F1, margin, ratio] = solveLP(r, 0*yaw, fg, [0 1 1 1], 20, 45, 0, 20)
% [F2, margin, ratio] = solveLP(r, 0*yaw, fg, [1 1 1 1], 20, 45, 0, 20)
% return
t = .1:.1:8;
forces = zeros(6, length(t));
margins = zeros(1, length(t));
for i = 1:length(t)
    mode = ceil(t(i))-1
    [F, margin, ratio] = solveLP(r, 0*yaw, fg, [0 1 1 0], 20, 45, 2, 20, mode);
    forces(1:3, i) = F(:, 1);
    forces(4:6, i) = F(:, 4);
    margins(i) = margin;
end

close all
plot(t, forces(1:3,:));
hold on;
plot(t, margins, '--');
legend('Lateral', 'Tangential', 'Normal', 'Margin', 'location', 'north');
xlabel('Step Progress');
ylabel('Force (N)');
title('Trot Step Sequence');
return

k = 0:1:45;
margins = zeros(size(k));
ratios = zeros(size(k));
taus = zeros(4, length(k));
angles = [repmat([0; -45; 30], 1, 4), [0; 0; -20]];
for i = 1:length(k)
    [F, margin, ratio] = solveLP(r, k(i)*yaw, fg, stance, 20, 45, 2, 20);
    tau = torques(F, angles, getJ);
    taus(1, i) = max(abs(tau(1, 1:4)));
    taus(2, i) = max(abs(tau(2, 1:4)));
    taus(3, i) = max(abs(tau(3, 1:4)));
    taus(4, i) = abs(tau(3, end));
    margins(i) = margin;
    ratios(i) = ratio;
end
close all
plot(k, ratios);
xlabel('Gripper Angle Offset');
ylabel('Adhesion Ratio');
title('Front Swing');
xlim([0, 45]);
figure
plot(k, margins);
xlabel('Gripper Angle Offset');
ylabel('Adhesion Margin (N)');
title('Front Swing');
xlim([0, 45]);
figure
plot(k, taus);
xlabel('Gripper Angle Offset');
ylabel('Motor Torque (Nm)');
legend('Yaw Motor', 'Shoulder Motor', 'Knee Motor', 'Tail Motor', 'location', 'Northwest');
title('Front Swing');
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

function [F, margin, ratio] = solveLP(r, yaw, wrench, stance, phi, theta, fmin, fmax, mode)
    % x = [F1 F2 F3 F4 N5 u] where Fi = [Bi Ti Ni]
    f = [zeros(13, 1); 1];
    Aeq = zeros(6, 14);
    beq = wrench;
    A = zeros(12, 14);
    H = zeros(14, 14);
    b = zeros(12, 1);
    lb = zeros(14, 1)-inf;
    ub = zeros(14, 1)+inf;
%     if mode < 1 || mode > 6
%         stance = [1 1 1 1];
%     end
    for i = 1:4
        % Force/torque balance
        R = [cosd(yaw(i)) -sind(yaw(i)) 0;
             sind(yaw(i)) cosd(yaw(i)) 0;
             0 0 1];
        Aeq(1:3, i*3-2:i*3) = R;
        Aeq(4:6, i*3-2:i*3) = skew(r(:, i))*Aeq(1:3, i*3-2:i*3);
        
        % Normal force limit
        A(i*3-2, i*3-2:i*3) = [0 -tand(phi) -1]*stance(i);  % -N - T tan(theta)
        A(i*3-2, end) = -1*stance(i);  % -u
        
        % Lateral force limit
        A(i*3-1, i*3-2:i*3) = [1 -tand(theta) 0]*stance(i);  % B - T tan(phi)
        A(i*3, i*3-2:i*3) = [-1 -tand(theta) 0]*stance(i);  % -B - T tan(phi)
        
        % Bounds
        lb(i*3-2:i*3) = [-fmax, fmin, -fmax]*stance(i);
        ub(i*3-2:i*3) = [fmax, fmax, fmax]*stance(i);
    end
    % Tail
    Aeq(1:3, end-1) = [0; 0; 1];
    Aeq(4:6, end-1) = skew(r(:, 5))*Aeq(1:3, end-1);
    lb(end-1) = 0;
    ub(end-1) = fmax;
    H = ones(size(H))*1e-3;
    
%     id = 1:3;
%     if mode == 1
%         lb(id) = [0;0;0];
%         ub(id) = lb(id);
%     elseif mode == 2
%         lb(id) = [0;-4;0];
%         ub(id) = lb(id);
%     elseif mode == 3
%         lb(id) = [0;0;0];
%         ub(id) = lb(id);
%     elseif mode == 4
%         lb(id) = [0;0;4];
%         ub(id) = lb(id);
%     elseif mode == 5
%         lb(id) = [0;4;4];
%         ub(id) = lb(id);
%     elseif mode == 6
%         lb(id) = [0;wrench(2)/4;0];
%         ub(id) = lb(id);
%     end
%     
%     id = 10:12;
%     if mode == 1
%         lb(id) = [0;0;0];
%         ub(id) = lb(id);
%     elseif mode == 2
%         lb(id) = [0;-4;0];
%         ub(id) = lb(id);
%     elseif mode == 3
%         lb(id) = [0;0;0];
%         ub(id) = lb(id);
%     elseif mode == 4
%         lb(id) = [0;0;4];
%         ub(id) = lb(id);
%     elseif mode == 5
%         lb(id) = [0;4;4];
%         ub(id) = lb(id);
%     elseif mode == 6
%         lb(id) = [0;wrench(2)/4;0];
%         ub(id) = lb(id);
%     end
    
    
    options = optimoptions('quadprog', 'Display', 'off');
%     [X,FVAL,EXITFLAG] = linprog(f,A,b,Aeq,beq,lb,ub);
    [X,FVAL,EXITFLAG] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options)
%     X = fmincon(@costfun,X,A,b,Aeq,beq,lb,ub);
    F = zeros(3, 5);
    F0 = zeros(3, 5);
    for i = 1:4
        R = [cosd(yaw(i)) -sind(yaw(i)) 0;
             sind(yaw(i)) cosd(yaw(i)) 0;
             0 0 1];
        F0(:, i) = X(i*3-2:i*3);
        F(:, i) = R*X(i*3-2:i*3);
    end
    F0(:, end) = [0; 0; 1]*X(end-1);
    F(:, end) = [0; 0; 1]*X(end-1);
    margin = -X(end);
    margins = A([1, 4, 7, 10], 1:end-1) * X(1:end-1);
    ratio = max(-F0(3, :)./F0(2, :));
    ratios = -F0(3, 1:2)./F0(2, 1:2);
end

function [F, margin, ratio] = solveLP1(r, yaw, wrench, stance, phi, theta, fmin, fmax)
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
    H = ones(size(H))*1e-3;
    options = optimoptions('quadprog', 'Display', 'off');
%     [X,FVAL,EXITFLAG] = linprog(f,A,b,Aeq,beq,lb,ub);
    [X,FVAL,EXITFLAG] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
%     X = fmincon(@costfun,X,A,b,Aeq,beq,lb,ub);
    F = zeros(3, 5);
    F0 = zeros(3, 5);
    for i = 1:4
        R = [cosd(yaw(i)) -sind(yaw(i)) 0;
             sind(yaw(i)) cosd(yaw(i)) 0;
             0 0 1];
        F0(:, i) = [1 0 0 0; 0 1 0 0; 0 0 1 -1]*X(i*4-3:i*4);
        F(:, i) = R*[1 0 0 0; 0 1 0 0; 0 0 1 -1]*X(i*4-3:i*4);
    end
    F0(:, end) = [0; 0; 1]*X(17);
    F(:, end) = [0; 0; 1]*X(17);
    margin = -X(end)
    margins = A([1, 4, 7, 10], 1:end-1) * X(1:end-1)
    ratio = max(-F0(3, :)./F0(2, :))
    ratios = -F0(3, 1:2)./F0(2, 1:2)
end
    
% Skew symmetric matrix
function vhat = skew(v)
    vhat = [0, -v(3), v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function c = costfun(x)
    c = -inf;
    for i = 1:4
        T = x(4*i-2);
        Nplus = x(4*i-1);
        Nminus = x(4*i);
        if Nminus > Nplus
            c = max(c, (Nminus-Nplus)/T);
        end
    end
%     c = c + x'*x*.0001;
end

function tau = torques(F, a, jacobian)
    a = deg2rad(a);
    Jtail = [0; 0; 350]/1000;
    tau = zeros(3, 5);
    for i = 1:4
        Jleg = jacobian(a(1, i), a(2, i), a(3, i))/1000;
        if any(i == [1, 3])
            F(1, i) = -F(1, i);
        end
        tau(:, i) = -Jleg'*F(:, i);
    end
    tau(3, 5) = -Jtail'*F(:, 5);
end