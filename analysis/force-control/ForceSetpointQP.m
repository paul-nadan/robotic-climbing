% F = [0, 20, 20, 0, 0, 0];
F = [0, 30, 0, -2000, 0, 0]';
% F = [0, 0, 20, 0, 0, 0]';
r = [211.5; 141.7; 94.2] .* [-1 1 -1 1 0; 1 1 -1 -1 0; -1 -1 -1 -1 0];
r(:,end) = [0; -471.7; 0];
% N = [-0.6 0.6 -.6 0.6; 0 0 0 0; .8 .8 .8 .8];
N = [0 0 0 0; 0 0 0 0; 1 1 1 1];
yaw = [-1 1 -1 1]*0;
% mirror = [1 -1 1 -1];
mirror = -[1 -1 1 -1];
weights = [1 1 1 1 1];

% T B+ N+ N-
% B T N
% x y z
% B T N

% function X = ForceSetpointQP(F, r, N, yaw, mirror, weights)

Fmin = 2;
amax = 20;
options = optimoptions('quadprog','Display','none');
H = eye(17)*.1;
f = zeros(17, 1);
A = zeros(8, 17);
b = zeros(8, 1);
Aeq = zeros(6, 17);
beq = F;
lb = zeros(17, 1);
ub = [inf; 0; inf; inf; inf; 0; inf; inf; inf; 0; inf; inf; inf; 0; inf; inf; inf];

for i = 1:4
    ind4 = i*4-3:i*4;
    ind2 = i*2-1:i*2;
    R = getR(N(:, i), yaw(i));
    rhat = skew(r(:, i));
    
    H(ind4(1:2), ind4(1:2)) = H(ind4(1:2), ind4(1:2)) + 1;
    H(ind4(4), ind4(4)) = H(ind4(4), ind4(4)) + 1;
    A(ind2, ind4) = -[1 -1 0 0; 
        tand(amax) tand(amax)/sqrt(2) 0 -1];
    b(ind2) = [-weights(i)*Fmin; 0];
    Aeq(1:3, ind4) = weights(i)*R*[0 mirror(i) 0 0; 1 0 0 0; 0 0 1 -1];
    Aeq(4:6, ind4) = rhat*Aeq(1:3, ind4);
end
Aeq(:, end) = weights(end)*[0; 0; 1; skew(r(:,end))*[0; 0; 1]];

[x,fval,exitflag,output] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options)

X = zeros(3, 5);
for i = 1:4
    R = getR(N(:, i), yaw(i));
    X(:,i) = weights(i)*R*[0 mirror(i) 0 0; 1 0 0 0; 0 0 1 -1]*x(i*4-3:i*4);
end
X(:,5) = [0; 0; x(end)]

plotResult(r, X, N, yaw, F, mirror)
if exitflag <= 0
    output
end
% end

function Xhat = skew(x)
    Xhat = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
end

% Get rotation from gripper frame to world frame
function R = getR(n, yaw)
    Ryaw = [cosd(yaw) sind(yaw) 0; -sind(yaw) cosd(yaw) 0; 0 0 1];
    x = cross([0; 1; 0], n);
    y = cross(n, x);
    Rn = [x y n];
    R = Rn * Ryaw;
end

function plotResult(r, X, N, yaw, F, mirror)
    figure(1);
    clf;
    plot3(r(1,:), r(2,:), r(3,:), '.', 'markersize', 30);
    hold on;
    quiver3(r(1,:), r(2,:), r(3,:), X(1,:), X(2,:), X(3,:));
    for i = 1:4
        Ri = getR(N(:, i), yaw(i));
        Ni = 100*Ri*[0; 0; 1];
        Ti = 100*Ri*[0; 1; 0];
        Bi = 100*Ri*[sqrt(.5)*mirror(i); sqrt(.5); 0];
        quiver3(r(1,i), r(2,i), r(3,i), Ni(1), Ni(2), Ni(3), 'k');
        quiver3(r(1,i), r(2,i), r(3,i), Ti(1), Ti(2), Ti(3), 'k');
        quiver3(r(1,i), r(2,i), r(3,i), Bi(1), Bi(2), Bi(3), 'k--');
    end
    axis equal;
    xlabel('x');
    ylabel('y');
    zlabel('z');
end