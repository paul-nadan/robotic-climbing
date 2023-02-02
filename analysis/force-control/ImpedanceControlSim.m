set(0,'defaultfigureposition',[400 100 900 750])

q0 = [1;0;0;0]; % body orientation
x0 = [0;0;0]; % body origin
mode = [1,1,1,1,1]; % feet in contact

x0 = (rand(3,1)-.5)*0.5;

ws = 0.5; % stance width/2
ds = 0.3; % stance length/2
hs = 0.05; % stance height
Fmax = [-1 1; -1 1; -1 1].*[1;1;1];
m = 2;
J = eye(3);
g = vrrotvec2mat([0,0,1,deg2rad(0)])*[0; -1; 0];

stance = [.5; .5; .5]; % stance width/2, length/2, and height
body = [0.2; 0.5; 0]; % body width, length, and height
r0 = [-1 1 -1 1; 1 1 -1 -1; -1 -1 -1 -1].*stance;
r0 = [r0, [0;-stance(2)*3;-stance(3)]];
r = vrrotvec2mat([rand(1,3), deg2rad(20)])*r0 + (rand(3,5)-.5)*.5;
r = r0;
[rref, ~] = estimateFeet(x0, q0, zeros(3,1), zeros(3,1), r);

dt = 0.001;
tmax = 5;
framerate = 15;
controlrate = 0.01; % update period in seconds
kprev = NaN; % previous control update index

t = 0:dt:tmax;
N = length(t);
x = zeros(3, N); % position
q = zeros(4, N); % orientation (robot to world)
v = zeros(3, N); % robot frame velocity
w = zeros(3, N); % robot frame angular velocity
F = zeros(3, 5, N-1); % robot frame contact forces
xref = zeros(3, N);
N3 = floor(N/3);
% xref(2,1:N3) = 3*(1:N3)/N*.2;
% xref(2,N3+1:2*N3) = 1*.2;
% xref(2,2*N3+1:3*N3) = (1-3*(1:N3)/N)*.2;
Fref = [-1 1 -1 1 0; 0 0 0 0 0; 0 0 0 0 0]*0;
x(:,1) = x0;
q(:,1) = q0;
frames = struct('cdata',{},'colormap',{});

close all
for k = 1:N-1
    if t(k) > 4
        mode(1) = t(k)-4;
    elseif t(k) > 3
        mode(1) = 0;
    elseif t(k) > 2
        mode(1) = 3-t(k);
    end
    rref = rref + dt*(r0-rref);
    [rhat, vhat] = estimateFeet(x(:,k), q(:,k), v(:,k), w(:,k), r);
    ghat = estimateGravity(q(:,k), g, false);
    if isnan(kprev) || t(k) - t(kprev) >= controlrate
        F(:,:,k) = getControl(rhat-xref(:,k), vhat, ghat, rref, g, mode) + Fref.*mode;
%         F(:,:,k) = getControlQP(rhat-xref(:,k), vhat, ghat, rref, g, mode) + Fref.*mode;
        F(:,:,k) = min(max(F(:,:,k), Fmax(:,1)), Fmax(:,2));
        F(3,5,k) = max(0, F(3,5,k));
        kprev = k;
    else
        F(:,:,k) = F(:,:,k-1);
    end
    
    if mod(k*framerate*dt,1) == 0
        figure(1);
        clf;
        plotRobot(x(:,k), q(:,k), r, body);
        plotForces(r, F(:,:,k), q(:,k), x(:,k), m*g)
        axis equal;
        drawnow();
        if isempty(frames)
            frames = [getframe(gcf)];
        else
            frames = [frames,getframe(gcf)];
        end
    end
    
    [x1, q1, v1, w1] = dynamics(x(:,k), q(:,k), v(:,k), w(:,k), ...
                                F(:,:,k), r, m, J, g, dt);
    x(:,k+1) = x1;
    q(:,k+1) = q1;
    v(:,k+1) = v1;
    w(:,k+1) = w1;
end
figure(2);
plotPosition(t, x);
figure(3);
subplot(2,2,1);
plotInput(t, F, 0);
subplot(2,2,2);
plotInput(t, F, 1);
subplot(2,2,3);
plotInput(t, F, 2);
subplot(2,2,4);
plotInput(t, F, 3);

function vhat = skew(v)
    vhat = [0, -v(3), v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end

function G = getG(x, r, mode)
    r1 = r(:,1)';
    r2 = r(:,2)';
    r3 = r(:,3)';
    r4 = r(:,4)';
    r5 = r(:,5)';
    x = x';
    z = zeros(1,3);
    if nargin < 3
        mode = ones(1,5);
    end
    G = [eye(3), eye(3), eye(3), eye(3);
         skew(r1-x), skew(r2-x), skew(r3-x), skew(r4-x);
         r2-r1, r1-r2, z, z;
         r3-r1, z, r1-r3, z;
         z, r3-r2, r2-r3, z;
         r4-r1, z, z, r1-r4;
         z, r4-r2, z, r2-r4;
         z, z, r4-r3, r3-r4];
    
    G = G(1:6,:);
    G = [G, [0;0;1;skew(r5-x)*[0;0;1]]];
    return
    
    internal = [1 2 4; 1 3 5; 2 3 6; 4 5 6];
    for leg = 1:4
        if mode(leg) < 1
            cols = leg*3-2:leg*3;
            rows = internal(leg,:)+6;
            for i = 1:3
                G(rows(i), cols(i)) = 1e6;
            end
        end
    end
end

function [x1, q1, v1, w1] = dynamics(x, q, v, w, F, r, m, J, g, dt)
   Q = quat2rotm(q');
   x1 = x + Q*v*dt;
   v1 = v + (sum(F,2)/m + Q'*g - cross(w, v))*dt;
   rb = Q'*r;
   tau = cross(rb(:,1), F(:,1)) + cross(rb(:,2), F(:,2)) + ...
         cross(rb(:,3), F(:,3)) + cross(rb(:,4), F(:,4)) + ...
         cross(rb(:,5), F(:,5));
   w1 = w + J\(tau - cross(w, J*w))*dt;
   q1 = q + q2L(q)*[0;w]/2*dt;
end

% Determine estimated foot locations and velocities in robot frame (3x4)
function [rhat, vhat] = estimateFeet(x, q, v, w, r)
    rhat = quat2rotm(q')'*(r-x);
    vhat = -v - skew(w)*r;
end

% Determine estimated gravity vector in robot frame (3x1)
function ghat = estimateGravity(q, g, accel)
    if accel
        ghat = quat2rotm(q')'*g; % accelerometer present
    else
        ghat = g; % no accelerometer present
    end
end

% Compute desired contact forces (3x4)
function X = getControlQP(rhat, vhat, ghat, r0, g0, mode)
    if nargin < 3
        mode = ones(5);
    end
    W = diag([ones(3,1)*mode(1);ones(3,1)*mode(2);
              ones(3,1)*mode(3);ones(3,1)*mode(4);mode(5)]);
    kp = 5;
    kd = 2;
%     F = -kp*(r0 - rhat) - kd*vhat; % decentralized control

    G = getG([0;0;0], rhat, mode);
%     G0 = getG([0;0;0], rhat);
    Ginv = W*pinv(G*W);%*(1-min(mode)) + pinv(G0)*min(mode);
    xf = reshape(r0(:,1:4)-rhat(:,1:4), 12, 1);
    vf = reshape(-vhat(:,1:4), 12, 1);
    xf = [xf; r0(3,5) - rhat(3,5)];
    vf = [vf; -vhat(3,5)];
    F = kp*(kp*Ginv'*xf + kd*Ginv'*vf) + [ghat; 0;0;0];
    N = [0 0 0 0; 0 0 0 0; 1 1 1 1];
    yaw = [-1 1 -1 1]*15;
    mirror = [-1 1 -1 1];
    
    X = ForceSetpointQP(F, r0, N, yaw, mirror, mode);
end

function F = getControl(rhat, vhat, ghat, r0, g0, mode)
    if nargin < 3
        mode = ones(5);
    end
    W = diag([ones(3,1)*mode(1);ones(3,1)*mode(2);
              ones(3,1)*mode(3);ones(3,1)*mode(4);mode(5)]);
    kp = 50;
    kd = 1;
%     F = -kp*(r0 - rhat) - kd*vhat; % decentralized control

    G = getG([0;0;0], rhat, mode);
%     G0 = getG([0;0;0], rhat);
    Ginv = W*pinv(G*W);%*(1-min(mode)) + pinv(G0)*min(mode);
    
    xf = reshape(r0(:,1:4)-rhat(:,1:4), 12, 1);
    vf = reshape(-vhat(:,1:4), 12, 1);
    xf = [xf; r0(3,5) - rhat(3,5)];
    vf = [vf; -vhat(3,5)];
    Fd = -kp*(Ginv*Ginv')*vf;
    Fp = -kp*(Ginv*Ginv')*xf;
    F = Fp*kp + Fd*kd;
%     F = [F; 0];
    F = [reshape(F(1:12), 3,4), [0;0;F(end)]];
end

function L = q2L(q)
    s = q(1);
    v = q(2:4);
    L = [s -v'; v s*eye(3)+skew(v)];
end

function plotRobot(x, q, r, body)
    Q = quat2rotm(q');
    c = [-1 1 1 -1; 1 1 -1 -1; 0 0 0 0].*body;
    c = Q*(x+c);
    fill3(c(1,:), c(2,:), c(3,:), 'b');
    hold on;
    plot3(r(1,:), r(2,:), r(3,:), 'r.', 'markersize', 50);
end

function plotForces(r, F, q, x, W)
    F = [quat2rotm(q')*F, W];
    r = [r, x];
    quiver3(r(1,:), r(2,:), r(3,:), F(1,:), F(2,:), F(3,:));
end

function plotPosition(t, x)
    plot(t, x');
    xlabel('Time (s)');
    ylabel('Position (m)');
    legend('x', 'y', 'z');
    title('Position');
end

function plotInput(t, F, component)
    if nargin < 3
        component = 0;
    end
    titles = ['X', 'Y', 'Z'];
    if component == 0
        plot(t(1:end-1), reshape(vecnorm(F), 5, [])');
        title('Magnitude');
    else
        plot(t(1:end-1), reshape(F(component,:,:), 5, [])');
        title([titles(component), '-Component']);
    end
    xlabel('Time (s)');
    ylabel('Force (N)');
    legend('1', '2', '3', '4', 'tail');
end