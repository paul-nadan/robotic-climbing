% Compute new robot state given previous state and current step number
function r = step2(r0, count0, grid, skip, fixedFootPlacements)
    simpleStep = 0;
    swingPos = [];
    if nargin < 5
        fixedFootPlacements = [];
    end
    if simpleStep && ~isempty(r0.seed)
        seed = r0.seed;
        r0 = state2robot(r0.seed, r0.config);
        r0.seed = seed;
    end

    global x_optimal f_optimal
    x_optimal = 0;
    f_optimal = -1;
%     if count0 >= 4
%         r0.config.limits([3,6],:) = [10,150;10,150];
%         r0.config.limits([7,9],:) = [-75,75;-75,75];
%     end
    horizon = r0.gait.horizon;
    weights = [1; 1; 1; 1];
    if horizon == -1
        weights = 1;
        horizon = 2;
        r0.gait.horizon = 2;
    end
    weights = weights(1:min(length(weights), horizon));
    count = count0:count0+horizon-1;
%     oneleg = skip > 2;
%     skip = mod(skip, 3);
    
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
        lb(ix0+10:ix0+nX) = lb(ix0+10:ix0+nX) + r0.config.gait.buffer(:,i(iStep));
        ub(ix0+10:ix0+nX) = ub(ix0+10:ix0+nX) - r0.config.gait.buffer(:,i(iStep));
    end
    if simpleStep
        swingPos = r0.vertices(:,logical(any(r0.gait.feet == 1,2).*~r0.gait.feet(:,i)));
%         swingPos = swingPos + sum(r0.gait.dx, 2);
        h = grid.params{4};
        for iSwingFoot = 1:size(swingPos,2)
            swingPos(2,iSwingFoot) = arclengthStep(swingPos(2,iSwingFoot), h, sum(r0.gait.dx(2,:)));
        end
    end
    if ~isempty(fixedFootPlacements)
        fixedFootPlacements = fixedFootPlacements(:,[2,4,6,8]);
        iSwingFoot = logical(any(r0.gait.feet == 1,2).*~r0.gait.feet(:,i));
        iSwingFoot = iSwingFoot(any(r0.gait.feet == 1,2));
        swingPos = fixedFootPlacements(:,iSwingFoot);
    end
    
    
%     if oneleg
%         mask = [zeros(1,9), 1,1,1, 2,2,2, 3,3, 4,4] ~= i(1);
%         
%     end
    
    % Preliminary force prediction
%     [F0,~,~,~] = quasiStaticDynamics(state2robot(Xn(1:length(X0)),r0.config), count0, grid);
%     F0 = reshape(F0(:,1:end-1), [], 1);
%     lb = [lb; zeros(size(F0))-Inf];
%     ub = [ub; zeros(size(F0))+Inf];
%     Aeq = [Aeq, zeros(horizon, length(F0))];
    
    % Preliminary solution with constant cost
    options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
        'Algorithm','sqp','ConstraintTolerance',1e-4,...
        'Display','none','SpecifyObjectiveGradient',true, ...
        'CheckGradients', false, 'FiniteDifferenceType', 'central', ...
        'OutputFcn', @saveBest);
    [xFree,~,flagFree,outputFree] = fmincon(@(x)costFull(x, []),Xn,[],[],...
        [],[],lb,ub,@(x)constraintsFull(x, rd, i, count0>0, grid, 0, skip, swingPos),options);
    Xn = xFree(1:length(Xn));
    
    % Seed solver with previous horizon results
    if 0 && isfield(r0, 'seed')
        h = min(horizon*nX, length(r0.seed));
        if h == length(Xn) && ~skip
            Xn(1:h) = r0.seed(1:h);
            for iStep = 1:horizon
                ix0 = nX*(iStep-1);
                lb(ix0+1:ix0+2) = Xn(ix0+1:ix0+2)-delta;
                ub(ix0+1:ix0+2) = Xn(ix0+1:ix0+2)+delta;
            end
%             plotTerrain(grid);
%             plotRobot(r0);
%             for iStep = 1:length(r0.seed)/nX
%                 r = state2robot(r0.seed(nX*(iStep-1)+1:nX*iStep), r0.config);
%                 plotRobot(r);
%             end
%             drawnow()
        end
    end
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
        if iStep == 1 && simpleStep
%             swingPos = robotFree.vertices(:,logical(any(robotFree.gait.feet == 1,2).*~robotFree.gait.feet(:,i)));
%             swingPos = r0.vertices(:,logical(any(r0.gait.feet == 1,2).*~r0.gait.feet(:,i)));
%             swingPos = swingPos + sum(r0.gait.dx, 2);
        end
    end
    
    % Full solution with grip margin cost
    nFC = length(F0)+length(C0);
    Aeq = [Aeq, zeros(horizon, nFC)];
    lb = [lb; zeros(nFC, 1)-Inf];
    ub = [ub; zeros(nFC, 1)+Inf];
    x_optimal = 0;
    f_optimal = -1;
    
    [x,~,flag,output] = fmincon(@(x)costFull(x, weights),[Xn;F0;C0],[],[],...
        [],[],lb,ub,@(x)constraintsFull(x, rd, i, count0>0, grid, length(weights), skip, swingPos),options);
    if flag <= 0 && f_optimal ~= -1
        x = x_optimal;
        flag = 1;
        fprintf('Overwriting infeasible solution\n');
    end
    if flag == 0 && output.constrviolation < options.ConstraintTolerance
        flag = 1;
    end
    % Fallback to preliminary solution
    if (length(weights)==1&&max(C0)<=1&&count0>0&&flag<=0) || (simpleStep&&flag<=0)
        fprintf('Falling back on no-cost solution\n');
        x = [xFree(1:length(Xn)); F0; C0];
        flag = flagFree;
    end
    
    % Post-processing and fallback to shorter horizon
    r = state2robot(x(1:nX), r0.config);
    r.skip = skip;
    r.F = reshape(x(end-nFC+1:end-nFC+length(F0)/length(weights)), 3, []);
    r.fail = count0>0&&flag<=0;
    if ~isempty(r0.seed)
        r.seed = xFree;
    else
        r.seed = x(1:length(xFree));
    end
    if r.fail && ~simpleStep
        fprintf('\n');
%         disp(output.message);
        if isfield(r0, 'seed')
            r.seed = r0.seed;
        end
        if 0 && length(weights) > 1
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
%         elseif skip == 2% && skip > 1 && skip <= 4
%             fprintf('One-legged step\n');
%             r0.R0 = vrrotvec2mat([0 0 1 -yawErr])*r0.R0;
%             r = step2(r0, count0, grid, skip+1);
        elseif r0.config.threshold < 35
            fprintf('Raising threshold: %d\n', r0.config.threshold+1);
            r0.R0 = vrrotvec2mat([0 0 1 -yawErr])*r0.R0;
            r0.config.threshold = r0.config.threshold + 2;
            r = step2(r0, count0, grid, 0);
            r.config.threshold = r0.config.threshold - 2;
%         else
%             fprintf('Trying global optimization\n');
%             [x,~,flag,~,~] = optimizeGlobal(Xn, F0, C0, rd, lb, ub, Aeq, beq, i, count0, options, grid, weights, 0);
% %             if flag ~= -1
% %                 x = [Xn; F0; C0];
% %             end
%             r = state2robot(x(1:nX), r0.config);
%             r.F = reshape(x(end-nFC+1:end-nFC+length(F0)/length(weights)), 3, []);
%             r.fail = flag ~= -1;
%             r.seed = x(nX+1:horizon*nX);
%             r.skip = 0;
        end
    else
%         r.seed = x(nX+1:horizon*nX);
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
function [c,ceq] = constraintsFull(x, rd, i, stepping, grid, costHorizon, skip, swingPos)    
    iF = mod(i+skip, size(rd(1).gait.angles, 2))+1; % next step in gait cycle
    nF = 3*sum(rd(2).gait.feet(:,iF(1)) > 0); % number of force variables
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
        [cX,ceqX] = constraintsKinematic(xi, r, r0, i(iStep), stepping, grid, swingPos);
        newFeet = r.vertices;
        if iStep > 1
            stance = r0.gait.feet(:,i(iStep)) > 0;
            ceqStance = oldFeet(:, stance) - newFeet(:, stance);
            ceq = [ceq; reshape(ceqStance, [], 1)];
        end
        oldFeet = newFeet;
        c = [c; cX];
        ceq = [ceq; ceqX];
        if iStep <= costHorizon
            xFi = xF(nF*(iStep-1)+1:nF*iStep);
            xCi = xC(iStep);
            [cF, ceqF] = constraintsForce(r, xFi, iF(iStep), grid);
            cC = constraintsCost(r, xFi, xCi, i(iStep)+skip, grid, ~isempty(swingPos));
            c = [c; cF; cC];
            ceq = [ceq; ceqF];
        end
        r0 = r;
    end
end

% Satisfy kinematic constraints for one step
function [c,ceq] = constraintsKinematic(x, r, r0, i, stepping, grid, swingPos)
    allFeet = r.vertices(:, sum(r0.gait.feet, 2) > 0);
    stanceFeet = r.vertices(:, r0.gait.feet(:,i) == 1);
    fixedFeet = r0.vertices(:, r0.gait.feet(:,i) == 1);
    swingFeet = r.vertices(:,logical(any(r.gait.feet == 1,2).*~r.gait.feet(:,i)));
%     iswingFeet = logical(any(r.gait.feet,2).*~r.gait.feet(:,i));
    ceq = [x(4:6)'*x(7:9);
           norm(x(4:6))-1;
           norm(x(7:9))-1;
           contact(allFeet, grid);
           stepping*(stanceFeet(1,:)-fixedFeet(1,:))';
%            stepping*(stanceFeet(1,1:end-1)-fixedFeet(1,1:end-1))';
           stepping*(stanceFeet(2,:)-fixedFeet(2,:))'];
    if ~isempty(swingPos)
        ceq = [ceq; reshape(swingFeet(1:2,:) - swingPos(1:2,:), [], 1)];
    end
    body = [r.bodies{:},r.vertices(:,~any(r.gait.feet,2))];
    dz = -contact(body, grid) + r0.config.clearance;
    B = cross(x(4:6), x(7:9));
%     dy = r.vertices(2, iswingFeet) - r0.vertices(2, iswingFeet) - 0.02;
    c = [dz; -B(3)];
%     c = [dz; -B(3); grid.params{3}*5-swingFeet(2)];
end

% Satisfy force and torque balance constraints for one step
function [c, ceq] = constraintsForce(robot, xF, iF, grid)
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    feet = robot.vertices(:, robot.gait.feet(:,iF) > 0);
    r = feet - robot.origin;
    
    % Find contact forces
    G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*9.81;
%     G = [1;0;0]*9.81;
    Aeq_torque = zeros(3,3*size(feet,2));
    for iFoot = 1:size(feet,2)
        ri = r(:,iFoot);
        Aeq_torque(:,iFoot*3-2:iFoot*3) = [0, ri(3), -ri(2);
                                           -ri(3), 0, ri(1);
                                           ri(2), -ri(1), 0];
    end
    gtorque = [0;0;0];
    for iBody = 1:length(robot.config.bodies)
        gvec = G*robot.config.mass(iBody);
        rg = robot.com(:,iBody) - robot.origin;
        gtorque = gtorque + cross(rg, gvec);
    end
    Aeq = [repmat(eye(3), 1, size(feet,2)); Aeq_torque];
    beq = [-G*sum(robot.config.mass); gtorque];
    ceq = Aeq*xF - beq;
    
    % Tail can only apply normal force
    F = reshape(xF, 3, []);
    Findices = robot.gait.feet(robot.gait.feet(:,iF) > 0, iF);
    F = F(:,Findices == 2);
    tails = robot.vertices(:, robot.gait.feet(:,iF) == 2);
    N = zeros(3,size(tails, 2));
    for iTail = 1:size(tails, 2)
        tail = tails(:,iTail);
        N(:,iTail) = [-f(tail(1),tail(2),grid.dzdx,grid);...
                       -f(tail(1),tail(2),grid.dzdy,grid); 1];
        N(:,iTail) = N(:,iTail)/norm(N(:,iTail));
    end
    Fnorm = zeros(1,size(tails,2));
    Ftang = zeros(1,size(tails,2));
    for iTail = 1:size(tails,2)
        Fnorm(iTail) = -F(:,iTail)'*N(:,iTail);
        Ftang(iTail) = norm(cross(F(:,iTail),N(:,iTail)));
    end
    c = [Fnorm; Ftang];
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
    [~, c1, c2] = gripperMargin(Fnorm, Ftang, robot.config.threshold);
    costs = [c1, c2];
end

% Global optimization with GlobalSearch
function [x,cost,flag,output,solutions] = optimizeGlobal(Xn, F0, C0, rd, lb, ub, Aeq, beq, i, count0, options, grid, weights, skip)
    gs = GlobalSearch('Display','iter', 'OutputFcn', @stopIfConverged);
    problem = createOptimProblem('fmincon','x0',[Xn;F0;C0],...
        'objective',@(x)costFull(x, weights),'lb',lb,'ub',ub,'Aeq',Aeq,'beq',beq,'nonlcon',...
        @(x)constraintsFull(x, rd, i, count0>0, grid, length(weights), skip, []),'options',options);
    
%     problem = createOptimProblem('fmincon','x0',Xn,...
%         'objective',@(x)costFull(x, []),'lb',lb(1:length(Xn)),'ub',ub(1:length(Xn)),'Aeq',Aeq,'beq',beq,'nonlcon',...
%         @(x)constraintsFull(x, rd, i, count0>0, grid, 0, skip),'options',options);
    
    [x,cost,flag,output,solutions] = run(gs,problem);
    output.constrviolation = 0;
end

function stop = stopIfConverged(optimValues,~)
    stop = ~isempty(optimValues.bestfval) && optimValues.bestfval <= 1+1e-5 ...
        && ~isempty(optimValues.localsolution.Exitflag) ...
        && optimValues.localsolution.Exitflag > 0;
end

function stop = saveBest(x,optimValues,~)
    global x_optimal f_optimal
    if (f_optimal == -1 || optimValues.fval < f_optimal) && optimValues.constrviolation < 1e-4
        x_optimal = x;
        f_optimal = optimValues.fval;
    end
    stop = 0;
end

function y = arclengthStep(x, r, dx)
    if x > 2*r
        arclen = pi*r+x-2*r;
    elseif x < 0
        arclen = x;
    else
        arclen = r*asin(x/r-1)+r*pi/2;
    end
    arclen = arclen + dx;
    if arclen < 0
        y = arclen;
    elseif arclen > pi*r
        y = arclen - pi*r + 2*r;
    else
        y = r*sin(arclen/r-pi/2)+r;
    end
end