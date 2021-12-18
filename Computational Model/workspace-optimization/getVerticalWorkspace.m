function [z, state, terrain] = getVerticalWorkspace(dx, dy, config, foot, direction)
    f = config.gait.feet(any(config.gait.feet,2),:);
    step = find(f(foot,:) == 0,1);
    x0 = [0;0;0; 0;1;0; -1;0;0; config.gait.angles(:,step)];
    r0 = state2robot(x0, config);
    r0 = moveRobot(r0, [0;0;-min(r0.vertices(3,:))], eye(3));
    x0 = robot2state(r0);
    
    delta = Inf;
    lb = [x0(1:2)-delta;-Inf;-1;-1;-1;-1;-1;-1;r0.config.limits(:,1)];
    ub = [x0(1:2)+delta;Inf;1;1;1;1;1;1;r0.config.limits(:,2)];
    
    options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
            'Algorithm','sqp','ConstraintTolerance',1e-4,...
            'Display','off','SpecifyObjectiveGradient',false, ...
            'CheckGradients', false);
    [state,fval,flag] = fmincon(@(x)costHeight(x,config,foot,direction),...
        x0,[],[], [],[],lb,ub,@(x)constraints(x,dx,dy,config,foot,r0),options);
    z = -fval*direction;
    if flag <= 0
        z = NaN;
    end
    terrain = 0;
end

function cost = costHeight(x, config, foot, direction)
    r = state2robot(x, config);
    feet = r.vertices(:, sum(config.gait.feet, 2) > 0);
    cost = -feet(3,foot)*direction;
end

function [c, ceq] = constraints(x, dx, dy, config, foot, r0)
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
           feet(2, foot) - feet0(2, foot) - dy];
       
    B = cross(x(4:6), x(7:9));
    c = [-B(3)];
end