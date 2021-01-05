% Compute new robot state given previous state and current step number
function r = step(r0, count0, grid)
    horizon = r0.gait.horizon;
    count = count0:count0+horizon-1;
    
    % Adjust heading
    i = mod(count-1, size(r0.gait.angles, 2))+1;
    targetHeading = 2*r0.origin(1);
    headingVec = r0.R0(:,2);
    heading = atan2(-headingVec(1), headingVec(2));
    yawErr = max(-r0.gait.dyaw, min(r0.gait.dyaw, targetHeading-heading));
    r0.R0 = vrrotvec2mat([0 0 1 yawErr])*r0.R0;
    
    % Initial guess and constraints
    rd = repmat(r0, horizon+1, 1); % Projected robot states starting at r0
    X0 = robot2state(r0); % Previous robot state vector
    Xn = zeros(length(X0)*horizon, 1); % Projected robot state vectors
    Aeq = zeros(horizon, length(Xn));
    beq = zeros(horizon, 1);
    delta = norm(r0.gait.dx(1:2,i(1)));
    lb = repmat([r0.origin(1:2)-delta;-Inf;-1;-1;-1;-1;-1;-1;r0.config.limits(:,1)], horizon, 1);
    ub = repmat([r0.origin(1:2)+delta;Inf;1;1;1;1;1;1;r0.config.limits(:,2)], horizon, 1);
    for iStep = 1:horizon
        rd(iStep+1).origin = rd(iStep).origin + rd(iStep).R0*r0.gait.dx(:,i(iStep));
        rd(iStep+1).angles = r0.gait.angles(:,i(iStep));
        ix0 = length(X0)*(iStep-1);
        Xn(ix0+1:ix0+length(X0)) = robot2state(rd(iStep+1));
        Aeq(iStep, (iStep-1)*length(X0)+4:(iStep-1)*length(X0)+6) = X0(7:9)';
        lb(ix0+1:ix0+2) = rd(iStep+1).origin(1:2)-delta;
        ub(ix0+1:ix0+2) = rd(iStep+1).origin(1:2)+delta;
    end
    
    % Seed solver with previous horizon results
    if isfield(r0, 'seed')
        h = min(horizon*length(X0), length(r0.seed));
        Xn(1:h) = r0.seed(1:h);
    end
    
    % Preliminary force prediction
    [F0,~,~,~] = quasiStaticDynamics(state2robot(Xn(1:length(X0)),r0.config), count0, grid);
    F0 = reshape(F0(:,1:end-1), [], 1);
    lb = [lb; zeros(size(F0))-Inf];
    ub = [ub; zeros(size(F0))+Inf];
    Aeq = [Aeq, zeros(horizon, length(F0))];
    
    % Preliminary solution with constant cost
    options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
        'Algorithm','sqp','ConstraintTolerance',1e-4,...
        'Display','off','SpecifyObjectiveGradient',true, 'CheckGradients', false, 'FiniteDifferenceType', 'central');
    [xFree,~,~,outputFree] = fmincon(@(x)costConstant(x,rd,i),[Xn;F0],[],[],...
        Aeq,beq,lb,ub,@(x)constraintsLookAhead(x,rd,i,count0>0,grid),options);
    Xn = xFree(1:length(Xn));
    robotFree = state2robot(Xn(1:length(X0)), r0.config);
    
    % Full force prediction
    [F0,~,~,~] = quasiStaticDynamicsMargin(robotFree,count0, grid);
    F0 = reshape(F0(:,1:end-1), [], 1);
    C0 = costGripper([Xn;F0],rd,i, grid);
    
    % Full solution with grip margin cost
    Aeq = [Aeq, zeros(size(Aeq,1),1)];
    lb = [lb; -Inf];
    ub = [ub; Inf];
%     x = [Xn; F0; C0];
%     output.constrviolation = 2;
    [x,~,~,output] = fmincon(@(x)costMax(x,rd,i),[Xn;F0;C0],[],[],...
        Aeq,beq,lb,ub,@(x)constraintsLookAheadMax(x,rd,i,count0>0, grid),options);

    % Fallback to preliminary solution
    if C0<=1&&count0>0&&output.constrviolation > options.ConstraintTolerance

        fprintf('Falling back on no-cost solution\n');
        x = [Xn; F0; C0];
        output = outputFree;
    end
    
    % Post-processing and fallback to shorter horizon
    r = state2robot(x(1:length(X0)), r0.config);
    r.F = reshape(x(end-length(F0)+1-1:end-1), 3, []);
    r.fail = count0>0&&output.constrviolation > options.ConstraintTolerance;
    if r.fail
        fprintf('\n');
%         disp(output.message);
        if isfield(r0, 'seed')
            r.seed = r0.seed;
        end
        if horizon > 1
            fprintf('Trying shorter horizon: %d\n', horizon-1);
            r0.R0 = vrrotvec2mat([0 0 1 -yawErr])*r0.R0;
            r0.gait.horizon = r0.gait.horizon - 1;
            r = step(r0, count0, grid);
            r.gait.horizon = horizon;
%         elseif norm(r0.gait.dx) > 2e-2 % try with half and quarter step
%             fprintf('Trying shorter step: %d\n', norm(r0.gait.dx(:,i(iStep)))/2);
%             r0.R0 = vrrotvec2mat([0 0 1 -yawErr])*r0.R0;
%             r0.gait.dyaw = r0.gait.dyaw/2;
%             r0.gait.dx = r0.gait.dx/2;
%             r0.horizon = 2; % TODO: shouldn't be hardcoded
%             r = step(r0, count0, grid);
%             r0.gait.dyaw = r0.gait.dyaw*2;
%             r0.gait.dx = r0.gait.dx*2;
        else
            fprintf('Trying global optimization\n');
            [x,cost,flag,output,solutions] = optimizeGlobal(Xn, F0, C0, rd, lb, ub, Aeq, beq, i, count0, options, grid);
            if cost > 1 || flag ~= -1
                x = [Xn; F0; C0];
            end
            r = state2robot(x(1:length(X0)), r0.config);
            r.F = reshape(x(end-length(F0)+1-1:end-1), 3, []);
            r.fail = cost > 1;
            r.seed = x(length(X0)+1:horizon*length(X0));
        end
%         else
%             x = [Xn; F0; C0];
%             r = state2robot(x(1:length(X0)), r0.config);
%             r.F = reshape(x(end-length(F0)+1-1:end-1), 3, []);
%             r.fail = true;
%             r.seed = x(length(X0)+1:horizon*length(X0));
%         end
    else
        r.seed = x(length(X0)+1:horizon*length(X0));
    end
end

function [c, g] = costMax(x, ~, ~)
    c = x(end);
    g = [zeros(length(x)-1,1); 1];
end

function [c,ceq] = constraintsLookAheadMax(x, rd, i, stepping, grid)
    [c0, ceq] = constraintsLookAhead(x(1:end-1), rd, i, stepping, grid);
    cost = costGripperMax(x, rd, i, grid);
    cslack = cost-x(end);
    cmargin = cost - 1;
    c = [c0;cslack';cmargin'];
end

% Gripper adhesion metric
function [c, g] = costGripperMax(x, rd, i, grid)
    x = x(1:end-1);
    iF = mod(i(1), size(rd(1).gait.angles, 2))+1; % next step in gait cycle
    nF = 3*sum(rd(2).gait.feet(:,iF) > 0); % number of force variables
    xF = x(end-nF+1:end); % force components
    Nx = (length(x)-nF)/(length(rd)-1); % length of robot state vector
    robot = state2robot(x(1:Nx), rd(1).config);
    feet = robot.vertices(:, robot.gait.feet(:,iF) > 0);
    
    F = reshape(xF, 3, []);
    
    % Find normal vectors
    N = zeros(3,size(feet, 2));
    for iFoot = 1:size(feet, 2)
        foot = feet(:,iFoot);
        N(:,iFoot) = [-f(foot(1),foot(2),grid.dzdx,grid);...
                       -f(foot(1),foot(2),grid.dzdy,grid); 1];
        N(:,iFoot) = N(:,iFoot)/norm(N(:,iFoot));
    end
    
    % Decompose forces into components
    Fnorm = zeros(1,size(feet,2));
    Ftang = zeros(1,size(feet,2));
    for iFoot = 1:size(feet,2)
        Fnorm(iFoot) = -F(:,iFoot)'*N(:,iFoot);
        Ftang(iFoot) = norm(cross(F(:,iFoot),N(:,iFoot)));
        Fnorm(iFoot) = max(0, Fnorm(iFoot));
    end
    
    % Compute margin
    [~, c1, c2] = gripperMargin(Fnorm, Ftang);
    c = [c1, c2];
    g = zeros(size(x));
end

% Magnitude of the total contact forces (F'*F)
function [c, g] = costForce(x, rd, i)
    iF = mod(i(1), size(rd(1).gait.angles, 2))+1; % next step in gait cycle
    nF = 3*sum(rd(2).gait.feet(:,iF) > 0); % number of force variables
    xF = x(end-nF+1:end); % force components
    c = xF'*xF;
    g = [zeros(length(x)-nF, 1); 2*xF];
end

% Set cost to zero and focus on satisfying constraints
function [c, g] = costConstant(x, rd, i)
    c = 0;
    g = zeros(size(x));
end

% Center of mass position error from default gait
function [c, g] = costKinematic(x, r0, ~)
    d = r0.origin(1:2) - x(1:2);
    c = max(norm(d),1e-12);
    g = [d(1)/c;
         d(2)/c;
         zeros(length(x)-2,1)];
end

function [c,ceq] = constraintsLookAhead(x, rd, i, stepping, grid)
    iF = mod(i(1), size(rd(1).gait.angles, 2))+1; % next step in gait cycle
    nF = 3*sum(rd(2).gait.feet(:,iF) > 0); % number of force variables
    xF = x(end-nF+1:end);
    Nx = (length(x)-nF)/(length(rd)-1);
    [cF, ceqF] = constraintsForce(xF, state2robot(x(1:Nx), rd(2).config), iF);
    [c1,ceq1] = constraints(x(1:Nx), rd(2), i(1), stepping, grid);
    ceq = zeros(length(ceq1)*(length(rd)-1), 1);
    c = zeros(length(c1)*(length(rd)-1), 1);
    ceq(1:length(ceq1)) = ceq1;
    c(1:length(c1)) = c1;
    for iStep = 2:length(rd)-1
        r = state2robot(x(Nx*(iStep-2)+1:Nx*(iStep-1)), rd(1).config);
        [cn,ceqn] = constraints(x(Nx*(iStep-1)+1:Nx*iStep), r, i(iStep), 1, grid);
        ceq(length(ceqn)*(iStep-1)+1:length(ceqn)*iStep) = ceqn;
        c(length(cn)*(iStep-1)+1:length(cn)*iStep) = cn;
    end
    c = [c; cF];
    ceq = [ceq; ceqF];
end

% x = current state, r0 = initial configuration (for stance feet)
function [c,ceq] = constraints(x, r0, i, stepping, grid)
    r = state2robot(x, r0.config);
    allFeet = r.vertices(:, sum(r0.gait.feet, 2) > 0);
    stanceFeet = r.vertices(:, r0.gait.feet(:,i) > 0);
    fixedFeet = r0.vertices(:, r0.gait.feet(:,i) > 0);
    ceq = [x(4:6)'*x(7:9);
           norm(x(4:6))-1;
           norm(x(7:9))-1;
           contact(allFeet, grid);
           stepping*(stanceFeet(1,:)-fixedFeet(1,:))';
           stepping*(stanceFeet(2,:)-fixedFeet(2,:))'];
    body = [r.bodies{:}];
    dz = -contact(body, grid) + r0.config.clearance;
    B = cross(x(4:6), x(7:9));
    c = [dz; -B(3)];
end

function [c,ceq] = constraintsForce(xF, robot, iF)
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    WEIGHT = 5*9.81; % Magnitude of gravity force (N)
    feet = robot.vertices(:, robot.gait.feet(:,iF) > 0);
    r = feet - robot.origin;
    
    % Find contact forces
    G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*WEIGHT;
    Aeq_torque = zeros(3,3*size(feet,2));
    for iFoot = 1:size(feet,2)
        ri = r(:,iFoot);
        Aeq_torque(:,iFoot*3-2:iFoot*3) = [0, ri(3), -ri(2);
                                           -ri(3), 0, ri(1);
                                           ri(2), -ri(1), 0];
    end
    Aeq = [repmat(eye(3), 1, size(feet,2)); Aeq_torque];
    beq = [-G;zeros(3,1)];
    ceq = Aeq*xF - beq;
    c = [];
end

% Global optimization with GlobalSearch
function [x,cost,flag,output,solutions] = optimizeGlobal(Xn, F0, C0, rd, lb, ub, Aeq, beq, i, count0, options, grid)
    gs = GlobalSearch('Display','iter', 'OutputFcn', @stopIfConverged);
    problem = createOptimProblem('fmincon','x0',[Xn;F0;C0],...
        'objective',@(x)costMax(x,rd,i),'lb',lb,'ub',ub,'Aeq',Aeq,'beq',beq,'nonlcon',@(x)constraintsLookAheadMax(x,rd,i,count0>0,grid),'options',options);
    [x,cost,flag,output,solutions] = run(gs,problem)
    output.constrviolation = 0;
end

function stop = stopIfConverged(optimValues,state)
    stop = ~isempty(optimValues.bestfval) && optimValues.bestfval <= 1;
end

% Global optimization with random initial seeds
function [x,cost,output] = optimizeRandom(Xn, F0, C0, rd, lb, ub, Aeq, beq, i, count0, options, N)
    [x,cost0,~,output] = fmincon(@(x)costMax(x,rd,i),[Xn;F0;C0],[],[],...
        Aeq,beq,lb,ub,@(x)constraintsLookAheadMax(x,rd,i,count0>0),options, grid);
    best = cost0;
    bestX = x;
    bestOutput = output;
    fprintf('Original solution cost = %f\n', cost0);

    offsetRange = (ub - lb);
    offsetRange(offsetRange>1000) = 1;
    for sample = 1:N
        lbNoInf = lb;
        lbNoInf(lb < -1000) = -0.5;
        offset = (rand(size(offsetRange))).*offsetRange + lbNoInf;
        [x,cost,~,output] = fmincon(@(x)costMax(x,rd,i),[offset(1:length(Xn));F0;C0]+offset,[],[],...
            Aeq,beq,lb,ub,@(x)constraintsLookAheadMax(x,rd,i,count0>0),options, grid);
        if output.constrviolation > options.ConstraintTolerance
            fprintf('Cost = failed\n');
            continue
        end
        fprintf('Cost = %f\n', cost);
        if cost < best
            best = cost;
            bestX = x;
            bestOutput = output;
        end
    end
    fprintf('Best solution cost = %f\n', best);
    fprintf('Improvement = %f\n', (cost0-best)/cost0);
    x = bestX;
    output = bestOutput;
end