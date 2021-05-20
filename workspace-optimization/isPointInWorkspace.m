function [success, state, terrain] = isPointInWorkspace(dx, dy, dz, config, foot, state0)
    f = config.gait.feet(any(config.gait.feet,2),:);
    step = find(f(foot,:) == 0,1);
    x0 = [0;0;0; 0;1;0; -1;0;0; config.gait.angles(:,step)];
    r0 = state2robot(x0, config);
    r0 = moveRobot(r0, [0;0;-min(r0.vertices(3,:))], eye(3));
    x0 = robot2state(r0);
    if nargin >= 6
        x0 = state0;
    end
    
    feet = r0.vertices(:, any(config.gait.feet == 1, 2));
    feet(3,:) = 0;
    feet(:,foot) = feet(:,foot) + [dx; dy; dz];
    [center, radius] = getSphere(feet);
    grid = sphereTerrain([-.5,.5], [-1,.5], 0.01, center, radius, 2*(dz>center(3))-1, [0;0;0]);
    terrain.center = center;
    terrain.radius = radius;
    terrain.sign = 2*(dz>center(3))-1;
    if dz*(dz-center(3)) > 0
        success = 0;
        state = x0;
        return
    end
    
    delta = Inf;
    lb = [x0(1:2)-delta;-Inf;-1;-1;-1;-1;-1;-1;config.limits(:,1)];
    ub = [x0(1:2)+delta;Inf;1;1;1;1;1;1;config.limits(:,2)];
    
    options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
            'Algorithm','sqp','ConstraintTolerance',1e-4,...
            'Display','off','SpecifyObjectiveGradient',true, ...
            'CheckGradients', false);
    [state,~,flag] = fmincon(@cost,...
        x0,[],[], [],[],lb,ub,@(x)constraints(x,dx,dy,dz,config,foot,r0,grid),options);
    success = flag > 0;
end

function [cost, grad] = cost(x)
    cost = 0;
    grad = 0*x;
end

function [c, ceq] = constraints(x, dx, dy, dz, config, foot, r0, terrain)
    f = config.gait.feet(any(config.gait.feet,2),:);
    step = find(f(foot,:) == 0,1);
    stance = f(:,step) == 1;
    tail = f(:,step) == 2;
    r = state2robot(x, config);
    feet = r.vertices(:, sum(config.gait.feet, 2) > 0);
    feet0 = r0.vertices(:, sum(config.gait.feet, 2) > 0);
    
    ceq = [x(4:6)'*x(7:9);
           norm(x(4:6))-1;
           norm(x(7:9))-1;
           (feet(1, stance) - feet0(1, stance))';
           (feet(2, stance) - feet0(2, stance))';
           feet(3, stance)';
           feet(3, tail)';
           feet(1, foot) - feet0(1, foot) - dx;
           feet(2, foot) - feet0(2, foot) - dy;
           feet(3, foot) - dz];
    
    body = [r.bodies{:},r.vertices(:,~any(r.gait.feet,2))];
    dz = -contact(body, terrain) + config.clearance;
    B = cross(x(4:6), x(7:9));
    c = [-B(3); dz];
end