close all
config = quadruped([3,3,2,2], 0.1, 0.3, {.2, [.16, .16]}, 0, 2);
seed = 42;
step = 1;
global ANIMATE RECORD
ANIMATE = 1;
RECORD = 1;
% grid = terrain([-.75, 1.25], ...
%         [-1.5 0.5], .01, [1,1,0.5], [1, .25, 0.0625], 0, [0;-.75;0], seed);
% r0 = spawnRobot(grid.spawn, eye(3), config, grid);
% r1 = step(r0, i, grid);

% plotTerrain(grid);
% plotRobot(r0);
% plotRobot(r1);

% X = [q1...qN, F1...FN]
t = 0:.05:1;
Ni = length(t);
Nq = size(config.gait.angles, 1) + 9;
Nf = 3*sum(config.gait.feet(:,step));
% x0 = getStartingPoint(r0, r1, step, grid, Nq, Nf, Ni, t);

state0 = robot2state(r0);
state1 = robot2state(r1);
lb = [repmat(min(state0,state1),Ni,1); zeros(Nf*Ni,1)-Inf];
ub = [repmat(max(state0,state1),Ni,1); zeros(Nf*Ni,1)+Inf];
lb(1:Nq) = state0;
ub(1:Nq) = state0;
lb(Nq*(Ni-1)+1:Nq*Ni) = state1;
ub(Nq*(Ni-1)+1:Nq*Ni) = state1;
    
options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
        'Algorithm','sqp','ConstraintTolerance',1e-4,...
        'Display','iter','SpecifyObjectiveGradient',true, 'CheckGradients', false, 'FiniteDifferenceType', 'central');
% [x1,~,~,output] = fmincon(@(x)cost0(x,r0,step,grid,Nq,Nf,Ni),x0,[],[],...
%         [],[],lb,ub,@(x)constraints(x,r0,r1,step,grid,Nq,Nf,Ni,t),options);
%     
% [x,~,~,output] = fmincon(@(x)cost(x,r0,step,grid,Nq,Nf,Ni),x1,[],[],...
%         [],[],lb,ub,@(x)constraints(x,r0,r1,step,grid,Nq,Nf,Ni,t),options);

FRAMES = animateTrajectory(x, step, config, grid, Nq, Nf, Ni);

function X0 = getStartingPoint(r0, r1, step, grid, Nq, Nf, Ni, t)
    x0 = robot2state(r0);
    x1 = robot2state(r1);
    X0 = zeros(Ni*(Nq+Nf),1);
    for it = 1:Ni
        X0(1+(it-1)*Nq:it*Nq) = x0 + (x1-x0)*it/Ni;
    end
    q = reshape(X0(1:Ni*Nq), Nq, Ni);
    [~, ddq] = computeDerivatives(zeros(Nq,1), q, t);
    for it = 1:Ni
        r = state2robot(q(:,it), r0.config);
        X0(1+(it-1)*Nf+Ni*Nq:it*Nf+Ni*Nq) = getForce(r, step, grid, ddq(:,it));
    end
end

function [c, g] = cost0(x, r0, i, grid, Nq, Nf, Ni)
    f = x(Ni*Nq+1:end);
    c = 0*sum(f.^2);
    g = 0*[zeros(Ni*Nq,1); 2*f];
    % TODO: switch to gripper adhesion
end

function [c, g] = cost(x, r0, i, grid, Nq, Nf, Ni)
    f = x(Ni*Nq+1:end);
    c = sum(f.^2);
    g = [zeros(Ni*Nq,1); 2*f];
    % TODO: switch to gripper adhesion
end

function [c,ceq] = constraints(x, r0, r1, i, grid, Nq, Nf, Ni, t)
    q = reshape(x(1:Ni*Nq), Nq, Ni);
    f = reshape(x(Ni*Nq+1:end), Nf, Ni);
    c = [];
    ceq = [];%[q(:,1) - robot2state(r0); q(:,end) - robot2state(r1)];
    [~, ddq] = computeDerivatives(zeros(Nq,1), q, t);
    for it = 1:Ni
        [ck, ceqk] = constraintsKinematic(q(:,it), r0, i, grid);
        ceqd = constraintsDynamic(q(:,it), f(:,it), r0.config, i, grid, ddq(:,it));
        c = [c; ck];
        ceq = [ceq; ceqk; ceqd];
    end
end

function [c,ceq] = constraintsKinematic(x, r0, i, grid)
    r = state2robot(x, r0.config);
    stanceFeet = r.vertices(:, r0.gait.feet(:,i) > 0);
    fixedFeet = r0.vertices(:, r0.gait.feet(:,i) > 0);
    ceq = [x(4:6)'*x(7:9);
           norm(x(4:6))-1;
           norm(x(7:9))-1;
           (stanceFeet(1,:)-fixedFeet(1,:))';
           (stanceFeet(2,:)-fixedFeet(2,:))';
           (stanceFeet(3,:)-fixedFeet(3,:))'];
    body = [r.bodies{:}];
    dz = -contact(body, grid) + r0.config.clearance;
    B = cross(x(4:6), x(7:9));
    c = [dz; -B(3)];
end

function ceq = constraintsDynamic(q, fvec, config, count, grid, ddq)
    robot = state2robot(q, config);

    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    WEIGHT = 5*9.81; % Magnitude of gravity force (N)
    
    i = mod(count, size(robot.gait.angles, 2))+1;
    feet = robot.vertices(:, robot.gait.feet(:,i) > 0);
    r = feet - robot.origin;
    
    % Find normal vectors
    N = zeros(3,size(feet, 2));
    for iFoot = 1:size(feet, 2)
        foot = feet(:,iFoot);
        N(:,iFoot) = [-f(foot(1),foot(2),grid.dzdx,grid);...
                       -f(foot(1),foot(2),grid.dzdy,grid); 1];
        N(:,iFoot) = N(:,iFoot)/norm(N(:,iFoot));
    end
    
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
    beq = [-G+ddq(1:3)*WEIGHT;zeros(3,1)];
    ceq = Aeq*fvec - beq;
    F = [reshape(fvec, 3, []), G];
    
    % Decompose forces into components
    Fnorm = zeros(1,size(feet,2));
    Ftang = zeros(1,size(feet,2));
    for iFoot = 1:size(feet,2)
        Fnorm(iFoot) = -F(:,iFoot)'*N(:,iFoot);
        Ftang(iFoot) = norm(cross(F(:,iFoot),N(:,iFoot)));
    end
    
    % Compute torques
    joints = robot.vertices - robot.links;
    T = zeros(size(joints));
    for iJoint = 1:size(robot.vertices, 2)
        nChildren = find([robot.config.parents(iJoint+1:end),0]<iJoint, 1);
        iChildren = iJoint:iJoint+nChildren-1;
        iFeetBool = double(robot.gait.feet(:,i)' > 0);
        iFeetBool(iChildren) = -iFeetBool(iChildren);
        iFeet = find(iFeetBool(iFeetBool~=0) < 0);
        for iFoot = iFeet
            rFoot = feet(:,iFoot)-joints(:,iJoint);
            T(:, iJoint) = T(:, iJoint) + cross(rFoot, F(:,iFoot));
        end
    end
end

function FRAMES = animateTrajectory(x, step, config, grid, Nq, Nf, Ni)
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    WEIGHT = 5*9.81; % Magnitude of gravity force (N)
    figure('units','normalized','outerposition',[0 0 .3 1]);
    FRAMES = struct('cdata',{},'colormap',{});
    q = reshape(x(1:Ni*Nq), Nq, Ni);
    f = reshape(x(Ni*Nq+1:end), Nf, Ni);
    for it = 1:Ni
        plotTerrain(grid);
        r = state2robot(q(:,it), config);
        plotRobot(r);
        G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*WEIGHT;
        F = [reshape(f(:,it), 3, []), G];
        plotForces(r, F, step-1, 'g', 0.016);
        drawnow();
        FRAMES(length(FRAMES)+1) = getframe(gcf);
    end
end

function [Fvec, T] = getForce(robot, count, grid, ddq)
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    WEIGHT = 5*9.81; % Magnitude of gravity force (N)
    
    i = mod(count, size(robot.gait.angles, 2))+1;
    feet = robot.vertices(:, robot.gait.feet(:,i) > 0);
    r = feet - robot.origin;
    
    % Find normal vectors
    N = zeros(3,size(feet, 2));
    for iFoot = 1:size(feet, 2)
        foot = feet(:,iFoot);
        N(:,iFoot) = [-f(foot(1),foot(2),grid.dzdx,grid);...
                       -f(foot(1),foot(2),grid.dzdy,grid); 1];
        N(:,iFoot) = N(:,iFoot)/norm(N(:,iFoot));
    end
    
    % Find contact forces
    G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*WEIGHT;
    Aeq_torque = zeros(3,3*size(feet,2));
    for iFoot = 1:size(feet,2)
        ri = r(:,iFoot);
        Aeq_torque(:,iFoot*3-2:iFoot*3) = [0, ri(3), -ri(2);
                                           -ri(3), 0, ri(1);
                                           ri(2), -ri(1), 0];
    end
    X0 = robot2state(robot);
    Aeq = [repmat(eye(3), 1, size(feet,2)); Aeq_torque];
    Aeq = [zeros(6,length(X0)), Aeq];
    beq = [-G+ddq(1:3)*WEIGHT;zeros(3,1)];
    [F0,~,~,~] = quasiStaticDynamics(robot, count, grid);
    F0 = reshape(F0(:,1:end-1), [], 1);
    lb = [X0; -ones(size(F0))*Inf];
    ub = [X0; ones(size(F0))*Inf];
    rd = [robot, robot];
    options = optimoptions('fmincon','Algorithm','sqp','Display','none');
    [X,~,~,~] = fmincon(@(x)costGripper(x, rd, count, grid),[X0;F0],[],[],...
        Aeq,beq,lb,ub,[],options);
    Fvec = X(length(X0)+1:end);
    F = [reshape(Fvec, 3, []), G];
    
    % Decompose forces into components
    Fnorm = zeros(1,size(feet,2));
    Ftang = zeros(1,size(feet,2));
    for iFoot = 1:size(feet,2)
        Fnorm(iFoot) = -F(:,iFoot)'*N(:,iFoot);
        Ftang(iFoot) = norm(cross(F(:,iFoot),N(:,iFoot)));
    end
    
    % Compute torques
    joints = robot.vertices - robot.links;
    T = zeros(size(joints));
    for iJoint = 1:size(robot.vertices, 2)
        nChildren = find([robot.config.parents(iJoint+1:end),0]<iJoint, 1);
        iChildren = iJoint:iJoint+nChildren-1;
        iFeetBool = double(robot.gait.feet(:,i)' > 0);
        iFeetBool(iChildren) = -iFeetBool(iChildren);
        iFeet = find(iFeetBool(iFeetBool~=0) < 0);
        for iFoot = iFeet
            rFoot = feet(:,iFoot)-joints(:,iJoint);
            T(:, iJoint) = T(:, iJoint) + cross(rFoot, F(:,iFoot));
        end
    end
end

function [dq, ddq] = computeDerivatives(dq0, q, t)
    dq = [dq0,diff(q,1,2)./diff(t,1)];
    ddq = [diff(dq,1,2)./diff(t,1), dq*0];
end