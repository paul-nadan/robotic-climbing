% close all
% k = -3;
% config = quadruped([3,3,2,2], 0.1, 0.3, {.2, [.16, .16]}, 0, 2);
% config.clearance = 0.1;
% f = [0.3, .4, -.3];
% % [r, f] = discreteStepp(f, k, 1, config, []);
% % figure();
% % [r, f] = discreteStepp(mirrorFoot(f), k, 3, config, []);
% 
% for step = 1:5
% % clf;
% figure()
% [r, f] = discreteStepp(f, k, mod(step-1, 4)+1, config, []);
% title("Step " + num2str(step));
% drawnow();
% r.fail
% end

% plotted angles: BR, FR, BL, FL

% angles: FL, BR, FR, BL

% Finds a valid robot state given the other foot positions (f), the
% surface curvature (k), the current step (step), the robot configuration
% (config), the initial guess (X0), and the gravity vector (g)
function [r, f] = discreteStep(f, k, step, config, X0)
    plotSphere(f, k)
    nextStep = mod(step, size(config.gait.feet, 2)) + 1;
    iFeet = config.gait.feet(:, step);
    if isempty(X0)
        r0 = getRobot(zeros(3,1), eye(3), config.gait.angles(:, step), config);
        feet = r0.vertices(:, iFeet);
        f2 = feet(:, 2) - feet(:, 1);
        yaw = atan2(f2(2), f2(1));
        r0 = moveRobot(r0, zeros(3,1), vrrotvec2mat([0, 0, 1, -yaw]));
        feet = r0.vertices(:, iFeet);
        r0 = moveRobot(r0, -feet(:,1), eye(3));
%         plotRobot(r0);
        X0 = robot2state(r0);
    end
    r0 = state2robot(X0, config);
    Aeq = zeros(1, length(X0));
    Aeq(4:6) = X0(7:9)';
    beq = 0;
    delta = 10*norm(config.gait.dx(1:2,step));
    lb = [r0.origin(1:2)-delta;-Inf;-1;-1;-1;-1;-1;-1;config.limits(:,1)];
    ub = [r0.origin(1:2)+delta;Inf;1;1;1;1;1;1;config.limits(:,2)];
    center = getSphere(f, k);
    f0 = [0, f(1), f(2); 0, 0, f(3); 0, 0, 0];
    options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
        'Algorithm','sqp','ConstraintTolerance',1e-4,...
        'Display','off','SpecifyObjectiveGradient',true, 'CheckGradients', false, 'FiniteDifferenceType', 'central');
    try
        [X,~,~,output] = fmincon(@(X)gaitCost(X,step,config),X0,[],[],...
        [],[],lb,ub,@(X)constraints(X,f0,center,k,step,config),options);
        r = state2robot(X, config);
        rplot = r;
        rplot = moveRobot(rplot, -center, eye(3));
        plotRobot(rplot);
        r.fail = output.constrviolation > options.ConstraintTolerance;
        nextStep = mod(step, size(config.gait.feet,2)) + 1;
        feet = r.vertices(:, config.gait.feet(:, nextStep));
        f = getFootCoordinates(feet(:,1), feet(:,2), feet(:,3));
    catch
        r = r0;
        r.fail = 1;
    end
end

% Penalizes distance from nominal gait
function [c, g] = gaitCost(X, nextStep, config)
    angles = X(10:end);
    dtheta = config.gait.angles(:, nextStep) - angles;
    dtheta = dtheta.*(config.parents==0)';
    c = sum(dtheta.^2);
    g = [zeros(9,1); -2*dtheta];
end

function [c,ceq] = constraints(X, f0, center, k, step, config)
    r = state2robot(X, config);
    nextStep = mod(step, size(config.gait.feet, 2)) + 1;
    stanceFeet = r.vertices(:, r.gait.feet(:,step) > 0);
    iSwingFeet = r.gait.feet(:,nextStep) > 0 & r.gait.feet(:,step) == 0;
    swingFeet = r.vertices(:, iSwingFeet);
    
    ceq = [X(4:6)'*X(7:9);
           norm(X(4:6))-1;
           norm(X(7:9))-1;
           getHeight(swingFeet, center, k);
           (stanceFeet(1,:)-f0(1,:))';
           (stanceFeet(2,:)-f0(2,:))';
           (stanceFeet(3,:)-f0(3,:))'];
    body = [r.bodies{:}];
    dz = -getHeight(body, center, k) + config.clearance;
    B = cross(X(4:6), X(7:9));
    c = [dz; -B(3)];
end

function h = getHeight(foot, center, k)
    x = foot(1) - center(1);
    y = foot(2) - center(2);
    z = foot(3) - center(3);
    h = z + sign(k)*sqrt(1/k^2 - x.^2 - y.^2);
    h(x.^2 + y.^2 > 1/k^2) = x.^2 + y.^2 + z.^2 - 1/k^2;
end

function center = getSphere(f, k)
    x = f(1)/2;
    y = (f(2)^2 + f(3)^2 - f(1)*f(2)) / (2*f(3));
    z = sign(k)*sqrt(1/k^2 - x^2 - y^2);
    center = [x; y; z];
end

function plotSphere(f, k)
    center = getSphere(f, k);
    [X,Y,Z] = sphere;
    X = X*(1/k) + 0*center(1);
    Y = Y*(1/k) + 0*center(2);
    Z = Z*(1/k) + 0*center(3);
    if ~isreal(center)
        return
    end
    surf(X, -Z, Y, 'FaceAlpha',.3);
    hold on;
    axis equal;
    plot3(-center(1), center(3), -center(2), 'r.', 'markersize', 30);
    plot3(f(1)-center(1), center(3), -center(2), 'r.', 'markersize', 30);
    plot3(f(2)-center(1), center(3), f(3)-center(2), 'r.', 'markersize', 30);
end

function f = getFootCoordinates(f1, f2, f3)
    f = zeros(1,3);
    f12 = f2 - f1;
    f13 = f3 - f1;
    f(1) = norm(f12);
    f(2) = f13'*f12/f(1);
    b = cross(f12, f13);
    n = cross(f12, b);
    f(3) = f13'*n/norm(n);
end

function f2 = mirrorFoot(f, step)
    f2 = zeros(1,3);
    f2(1) = sqrt(f(2)^2+f(3)^2);
end