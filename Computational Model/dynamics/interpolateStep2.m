close all
addpath('terrain-generation', 'config-generation', 'robot-kinematics', ...
    'optimization', 'visualization', 'discrete-model', 'dynamics');

global ANIMATE RECORD PLOT FRAMES
ANIMATE = 1;
RECORD = 1;
PLOT = 1;
FRAMES = struct('cdata',{},'colormap',{});

SIMULATE = ~~0;
INTERPOLATE = 1;

sample = 2;
config = simpleWalker('000100000', 0.2, 0.5, 0.1);
% config = simpleWalker('0001010100', 0.2, 0.5, 0.1);
n = 20;

if SIMULATE
    
    grid = terrain([-1.5, 1.5], [-1.5 3.5], .01, 1*[1,1,0.5], ...
        [1, .25, 0.0625], 0, [0;-.5;0], 42);
else
    for iRobot = 1:length(robots)
        if isequal(abs(robots{iRobot,1}(1).config.joints)>0, abs(config.joints)>0)
            r = robots{iRobot,sample};
            break
        end
    end
    grid = terrain([-1.5, 1.5], [-1.5 3.5], .01, 1*[1,1,0.5], ...
        [1, .25, 0.0625], 0, [0;-.5;0], seeds(1,1,sample));
end


if INTERPOLATE
    X = cell(length(r)-1);
end
skips = 0;
for i = 1:length(r)-1
    if INTERPOLATE
        skips = skips + r(i+1).skip;
        tic()
        X{i} = interpolate(r(i), r(i+1), grid, i+skips, n);
        toc()
    end
    animateInterpolatedStep(X{i}, r(1).config, grid);
end

function X = interpolate(r0, r1, grid, step, n)
    x0 = robot2state(r0);
    x1 = robot2state(r1);
    X = zeros(length(x0), n);
    dx = ones(size(x0))*250/n;
    dx(1:3) = 0.2/n;
    dx(4:9) = 1/n;
    step = mod(step-1, size(r0.gait.feet,2)) + 1;
    feet = r0.vertices(:, r0.gait.feet(:,step) == 1);
    xi = x0;
    xprev = x0;
    for i = 1:n
        options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
            'Algorithm','sqp','ConstraintTolerance',1e-4, 'display', 'none', ...
            'SpecifyObjectiveGradient',false, 'CheckGradients', false, ...
            'FiniteDifferenceType', 'central');
        lb = min(xprev-dx, xprev+dx);
        ub = max(xprev-dx, xprev+dx);
        p2 = [x0(1); x0(2)+(i-1)*.1/(n-1); x0(3)];
%         [xi,~,flag,output] = fmincon(@(x)interpolateCost2(x,state2robot(xi, r0.config)),xi,[],[],[],[],[],[],...
%             @(x)interpolateConstraints(x, feet, r0.config, grid, step),options);
        [xi,~,flag,output] = fmincon(@(x)interpolateCost(x,xi,dx),xi,[],[],[],[],[],[],...
            @(x)interpolateConstraints(x, p2, feet, r0.config, grid, step),options);
        if flag < 1
%             disp(output.message)
            if output.constrviolation > 1e-6
                disp(output.message)
                disp(output.constrviolation);
            end
        end
        X(:,i) = xi;
        xprev = xi;
%         xi = xi + (x1-xi)/(n-i);
    end
end

function [cost, g] = interpolateCost(x, xi, dx)
    error = (x-xi);
    cost = error'*error;
    g = 2*error;
end

function cost = interpolateCost2(x, r0)
    r = state2robot(x, r0.config);
    feet = r.vertices(:, any(r0.gait.feet == 1, 2));
    feet0 = r0.vertices(:, any(r0.gait.feet == 1, 2));
    error = reshape(feet - feet0, [], 1);
    cost = error'*error;
end

function [c,ceq] = interpolateConstraints(x, p2, stanceFeet, config, grid, step)
    r = state2robot(x, config);
    feet = r.vertices(:, config.gait.feet(:,step) == 1);
    ceq = [x(4:6)'*x(7:9);
           norm(x(4:6))-1;
           norm(x(7:9))-1;
           feet(1,:)'-stanceFeet(1,:)';
           feet(2,:)'-stanceFeet(2,:)';
           feet(3,:)'-stanceFeet(3,:)';
           p2(1)-x(1);
           p2(3)-x(3);
           p2(2)-x(2)];
    body = [r.bodies{:},r.vertices(:,~r.gait.feet(:,step))];
    dz = -contact(body, grid) + config.clearance;
    B = cross(x(4:6), x(7:9));
    c = [dz; -B(3)];
end

function animateInterpolatedStep(X, config, grid)
    global RECORD FRAMES
    plotTerrain(grid);
    for t = 1:size(X,2)
        rt = state2robot(X(:,t), config);
        g.feet = plotPoints(rt.vertices(:,any(config.gait.feet' == 1)), 'b.');        
        for iBody = 1:length(rt.bodies)
            body = rt.bodies{iBody};
            g.bodies(iBody) = fill3(body(1,:), -body(3,:), body(2,:), 'r');
        end
        joints = rt.vertices - rt.links;
        for iVertex = 1:size(rt.vertices,2)
            g.links(iVertex) = plotLine(joints(:,iVertex),...
                rt.vertices(:,iVertex), 'b');
        end
        drawnow();
        if RECORD
            if isempty(FRAMES)
                FRAMES = [getframe(gcf)];
            else
                FRAMES = [FRAMES,getframe(gcf)];
            end
        end
        if t < size(X,2)
            for iBody = 1:length(g.bodies)
                delete(g.bodies(iBody));
            end
            for iLink = 1:length(g.links)
                delete(g.links(iLink));
            end
            delete(g.feet);
        end
    end
end