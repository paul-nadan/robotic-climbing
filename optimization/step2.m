% Compute new robot state given previous state and current step number
function r = step2(r0, count0, grid, skip)
    horizon = r0.gait.horizon;
    weights = [1; 1];
    if horizon == -1
        weights = 1;
        horizon = 2;
        r0.gait.horizon = 2;
    end
    weights = weights(1:min(length(weights), horizon));
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
    nX = length(X0); % Number of state variables
    Xn = zeros(nX*horizon, 1); % Projected robot state vectors
    Aeq = zeros(horizon, length(Xn));
    beq = zeros(horizon, 1);
    delta = norm(r0.gait.dx(1:2,i(1)));
    lb = repmat([r0.origin(1:2)-delta;-Inf;-1;-1;-1;-1;-1;-1;r0.config.limits(:,1)], horizon, 1);
    ub = repmat([r0.origin(1:2)+delta;Inf;1;1;1;1;1;1;r0.config.limits(:,2)], horizon, 1);
    for iStep = 1:horizon
        rd(iStep+1).origin = rd(iStep).origin + rd(iStep).R0*r0.gait.dx(:,i(iStep));
        rd(iStep+1).angles = r0.gait.angles(:,i(iStep));
        ix0 = nX*(iStep-1);
        Xn(ix0+1:ix0+nX) = robot2state(rd(iStep+1));
        Aeq(iStep, (iStep-1)*nX+4:(iStep-1)*nX+6) = X0(7:9)';
        lb(ix0+1:ix0+2) = rd(iStep+1).origin(1:2)-delta;
        ub(ix0+1:ix0+2) = rd(iStep+1).origin(1:2)+delta;
    end
    
    % Seed solver with previous horizon results
    if isfield(r0, 'seed')
        h = min(horizon*nX, length(r0.seed));
        Xn(1:h) = r0.seed(1:h);
    end
    
    % Preliminary force prediction
%     [F0,~,~,~] = quasiStaticDynamics(state2robot(Xn(1:length(X0)),r0.config), count0, grid);
%     F0 = reshape(F0(:,1:end-1), [], 1);
%     lb = [lb; zeros(size(F0))-Inf];
%     ub = [ub; zeros(size(F0))+Inf];
%     Aeq = [Aeq, zeros(horizon, length(F0))];
    
    % Preliminary solution with constant cost
    options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
        'Algorithm','sqp','ConstraintTolerance',1e-4,...
        'Display','off','SpecifyObjectiveGradient',true, 'CheckGradients', false, 'FiniteDifferenceType', 'central');
    [xFree,~,~,outputFree] = fmincon(@(x)costFull(x, []),Xn,[],[],...
        Aeq,beq,lb,ub,@(x)constraintsFull(x, rd, i, count0>0, grid, 0, skip),options);
    Xn = xFree(1:length(Xn));
    
    % Full force prediction
    F0 = [];
    C0 = [];
    for iStep = 1:length(weights)
        robotFree = state2robot(Xn(nX*(iStep-1)+1:nX*iStep), r0.config);
        [f0,~,~,~] = quasiStaticDynamicsMargin(robotFree,count0+iStep-1, grid);
        f0 = reshape(f0(:,1:end-1), [], 1);
        c0 = max(costGripper(robotFree, f0, i(iStep)+skip, grid));
        F0 = [F0; f0];
        C0 = [C0; c0];
    end
    
    % Full solution with grip margin cost
    nFC = length(F0)+length(C0);
    Aeq = [Aeq, zeros(horizon, nFC)];
    lb = [lb; zeros(nFC, 1)-Inf];
    ub = [ub; zeros(nFC, 1)+Inf];
    [x,~,~,output] = fmincon(@(x)costFull(x, weights),[Xn;F0;C0],[],[],...
        Aeq,beq,lb,ub,@(x)constraintsFull(x, rd, i, count0>0, grid, length(weights), skip),options);

    % Fallback to preliminary solution
    if length(weights)==1&&max(C0)<=1&&count0>0&&output.constrviolation > options.ConstraintTolerance
        fprintf('Falling back on no-cost solution\n');
        x = [Xn; F0; C0];
        output = outputFree;
    end
    
    % Post-processing and fallback to shorter horizon
    r = state2robot(x(1:nX), r0.config);
    r.skip = skip;
    r.F = reshape(x(end-nFC+1:end-nFC+length(F0)/length(weights)), 3, []);
    r.fail = count0>0&&output.constrviolation > options.ConstraintTolerance;
    if r.fail
        fprintf('\n');
%         disp(output.message);
        if isfield(r0, 'seed')
            r.seed = r0.seed;
        end
        if length(weights) > 1
            fprintf('Trying shorter horizon: %d\n', -1);
            r0.R0 = vrrotvec2mat([0 0 1 -yawErr])*r0.R0;
            r0.gait.horizon = -1;
            r = step2(r0, count0, grid, 0);
            r.gait.horizon = horizon;
        elseif horizon > 1
            fprintf('Trying shorter horizon: %d\n', horizon-1);
            r0.R0 = vrrotvec2mat([0 0 1 -yawErr])*r0.R0;
            r0.gait.horizon = r0.gait.horizon - 1;
            r = step2(r0, count0, grid, 0);
            r.gait.horizon = horizon;
        elseif skip <= 1
            fprintf('Skipping step\n');
            r0.R0 = vrrotvec2mat([0 0 1 -yawErr])*r0.R0;
            r = step2(r0, count0, grid, skip+1);
        else
            fprintf('Trying global optimization\n');
            [x,~,flag,~,~] = optimizeGlobal(Xn, F0, C0, rd, lb, ub, Aeq, beq, i, count0, options, grid, weights, 0);
%             if flag ~= -1
%                 x = [Xn; F0; C0];
%             end
            r = state2robot(x(1:nX), r0.config);
            r.F = reshape(x(end-nFC+1:end-nFC+length(F0)/length(weights)), 3, []);
            r.fail = flag ~= -1;
            r.seed = x(nX+1:horizon*nX);
            r.skip = 0;
        end
    else
        r.seed = x(nX+1:horizon*nX);
    end
end

% Optimize gripper adhesion margin
function [c, g] = costFull(x, weights)
    horizon = length(weights);
    c = sum(x(end-horizon+1:end).*weights);
    g = zeros(size(x));
    g(end-horizon+1:end) = weights;
end

% Satisfy all constraints for all steps in simulation horizon
function [c,ceq] = constraintsFull(x, rd, i, stepping, grid, costHorizon, skip)    
    iF = mod(i(1)+skip, size(rd(1).gait.angles, 2))+1; % next step in gait cycle
    nF = 3*sum(rd(2).gait.feet(:,iF) > 0); % number of force variables
    X = x(1:end-costHorizon*(1+nF)); % state variables
    xF = x(end-costHorizon*(1+nF)+1:end); % force variables
    xC = x(end-costHorizon+1:end); % cost variables
    nX = length(X)/(length(rd)-1); % number of state variables
    
    c = [];
    ceq = [];
    r0 = rd(2);
    for iStep = 1:length(rd)-1
        xi = X(nX*(iStep-1)+1:nX*iStep);
        r = state2robot(xi, rd(1).config);
        [cX,ceqX] = constraintsKinematic(xi, r, r0, i(iStep), stepping, grid);
        c = [c; cX];
        ceq = [ceq; ceqX];
        if iStep <= costHorizon
            xFi = xF(nF*(iStep-1)+1:nF*iStep);
            xCi = xC(iStep);
            ceqF = constraintsForce(r, xFi, iF);
            cC = constraintsCost(r, xFi, xCi, i(iStep)+skip, grid, 0 > 1);
            c = [c; cC];
            ceq = [ceq; ceqF];
        end
        r0 = r;
    end
end

% Satisfy kinematic constraints for one step
function [c,ceq] = constraintsKinematic(x, r, r0, i, stepping, grid)
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

% Satisfy force and torque balance constraints for one step
function ceq = constraintsForce(robot, xF, iF)
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
end

% Satisfy cost slack variable constraint for one step
function c = constraintsCost(robot, xF, xC, i, grid, unconstrained)
    costs = costGripper(robot, xF, i, grid);
    cslack = costs - xC;
    cmargin = costs - 1;
    if unconstrained
        c = cslack';
    else
        c = [cslack';cmargin'];
    end
end

% Gripper adhesion metric
function costs = costGripper(robot, xF, i, grid)
    iF = mod(i, size(robot.gait.angles, 2))+1; % next step in gait cycle
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
    costs = [c1, c2];
end

% Global optimization with GlobalSearch
function [x,cost,flag,output,solutions] = optimizeGlobal(Xn, F0, C0, rd, lb, ub, Aeq, beq, i, count0, options, grid, weights, skip)
    gs = GlobalSearch('Display','iter', 'OutputFcn', @stopIfConverged);
    problem = createOptimProblem('fmincon','x0',[Xn;F0;C0],...
        'objective',@(x)costFull(x, weights),'lb',lb,'ub',ub,'Aeq',Aeq,'beq',beq,'nonlcon',...
        @(x)constraintsFull(x, rd, i, count0>0, grid, length(weights), skip),'options',options);
    [x,cost,flag,output,solutions] = run(gs,problem);
    output.constrviolation = 0;
end

function stop = stopIfConverged(optimValues,~)
    stop = ~isempty(optimValues.bestfval) && optimValues.bestfval <= 1 ...
        && ~isempty(optimValues.localsolution.Exitflag) ...
        && optimValues.localsolution.Exitflag > 0;
end