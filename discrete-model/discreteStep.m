% Finds a valid robot state given the other foot positions (f), the
% surface curvature (k), the current step (step), the robot configuration
% (config), the initial guess (X0), and the gravity vector (g)
function r = discreteStep(f, k, step, config, X0)
    plotSphere(f, k)
    nextStep = mod(step, size(config.gait.feet, 2)) + 1;
    iFeet = config.gait.feet(:, step);
    if isempty(X0)
        r0 = getRobot(zeros(3,1), eye(3), config.gait.angles(:, nextStep), config);
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
        [X,~,~,output] = fmincon(@(X)gaitCost(X,nextStep,config),X0,[],[],...
        [],[],lb,ub,@(X)constraints(X,f0,center,k,step,config),options);
        r = state2robot(X, config);
        plotRobot(r);
        r2 = r
        r.fail = output.constrviolation > options.ConstraintTolerance;
    catch
        r = r0;
        r.fail = 1;
    end
end

% Penalizes distance from nominal gait
function [c, g] = gaitCost(X, nextStep, config)
    angles = X(10:end);
    dtheta = config.gait.angles(:, nextStep) - angles;
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
    h = foot(3) - center(3) + sign(k)*sqrt(1/k^2 - x.^2 - y.^2);
    h(x.^2 + y.^2 > 1/k^2) = NaN;
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
    X = X*(1/k) + center(1);
    Y = Y*(1/k) + center(2);
    Z = Z*(1/k) + center(3);
    if ~isreal(center)
        return
    end
    surf(X, -Z, Y, 'FaceAlpha',.3);
    hold on;
    axis equal;
    plot3(0, 0, 0, 'r.', 'markersize', 30);
    plot3(f(1), 0, 0, 'r.', 'markersize', 30);
    plot3(f(2), 0, f(3), 'r.', 'markersize', 30);
end