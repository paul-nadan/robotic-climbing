close all
addpath('terrain-generation', 'config-generation', 'robot-kinematics', ...
    'optimization', 'visualization', 'discrete-model', 'dynamics');
config = quadruped([3,3,2,2], 0.1, 0.3, {.2, [.16, .16]}, 0, 2);
seed = 42;
iStep = 2;
global ANIMATE RECORD
ANIMATE = 1;
RECORD = 1;
% grid = terrain([-.75, 1.25], ...
%         [-1.5 0.5], .01, [1,1,0.5], [1, .25, 0.0625], 0, [0;-.75;0], seed);
% r0 = spawnRobot(grid.spawn, eye(3), config, grid);
% r1 = step(r0, 1, grid);
% r2 = step(r1, iStep, grid);
% 
% plotTerrain(grid);
% plotRobot(r1);
% plotRobot(r2);

dt = 0.05;
t = 0:dt:1;
N = length(t);
Nq = size(config.gait.angles, 1) + 9;
Nf = 3*sum(config.gait.feet(:,iStep));
q0 = robot2state(r1);
q1 = robot2state(r2);
% x0 = getStartingPoint(q0, q1, config, iStep, grid, Nq, Nf, N, dt);

lb = [repmat(min(q0,q1)',N,1), zeros(N,Nq+Nf)-Inf];
ub = [repmat(max(q0,q1)',N,1), zeros(N,Nq+Nf)+Inf];
lb(1,1:2*Nq) = [q0', zeros(1,Nq)];
ub(1,1:2*Nq) = [q0', zeros(1,Nq)];
lb(end,1:2*Nq) = [q1', zeros(1,Nq)];
ub(end,1:2*Nq) = [q1', zeros(1,Nq)];
% lb(end,Nq+1:2*Nq) = [zeros(1,Nq)];
% ub(end,Nq+1:2*Nq) = [zeros(1,Nq)];
    
% options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
%         'Algorithm','sqp','ConstraintTolerance',1e-4,...
%         'Display','iter','SpecifyObjectiveGradient',true, 'CheckGradients', false, 'FiniteDifferenceType', 'central');
% [x1,~,~,output] = fmincon(@(x)cost0(x,r1,iStep,grid,Nq,Nf,N),x0,[],[],...
%         [],[],lb,ub,@(x)constraints(x,r1,r2,iStep,grid,Nq,Nf,N,dt),options);
% % 
options = optimoptions('fmincon','MaxFunctionEvaluations',2e4,...
        'Algorithm','sqp','ConstraintTolerance',1e-4,...
        'Display','iter','SpecifyObjectiveGradient',false, 'FiniteDifferenceType', 'central');
[x,~,~,output] = fmincon(@(x)costGrip(x,r1,iStep,grid,Nq,Nf,N),x1,[],[],...
        [],[],lb,ub,@(x)constraints(x,r1,r2,iStep,grid,Nq,Nf,N,dt),options);

[cFinal, costFinal] = costGrip(x, r1, iStep, grid, Nq, Nf, N);
FRAMES = animateTrajectory(x, iStep, config, grid, Nq, Nf, N);

function x0 = getStartingPoint(q0, q1, config, step, grid, Nq, Nf, N, dt)
    x0 = zeros(N,Nq*2+Nf);
    a = 2*(q1-q0)/(N-1).^2;
    for n = 1:N
        if n <= 1+(N-1)/2
            q = q0 + a*(n-1).^2;
            dq = 2*a*(n-1)/dt;
            ddq = 2*a/dt^2;
            r = state2robot(q, config);
            f = getForce(r, step, grid, ddq);
            x0(n,:) = [q', dq', f'];
        else
            q = q1 - a*(N-n).^2;
            dq = 2*a*(N-n)/dt;
            ddq = -2*a/dt^2;
            r = state2robot(q, config);
            f = getForce(r, step, grid, ddq);
            x0(n,:) = [q', dq', f'];
        end
    end
end

function [c, g] = cost0(x, r0, i, grid, Nq, Nf, N)
    c = 0;
    g = zeros(size(x));
end

function [c, g] = cost(x, r0, i, grid, Nq, Nf, N)
    f = x(:,2*Nq+1:end);
    c = sum(f.^2, 'all');
    g = [zeros(N,Nq*2), 2*f];
    % TODO: switch to gripper adhesion
end

function [c, g] = costGrip(x, r0, iStep, grid, Nq, Nf, N)
    iF = mod(iStep, size(r0.gait.angles, 2))+1; % next step in gait cycle
    feet = r0.vertices(:, r0.gait.feet(:,iF) > 0);    
    
    % Find normal vectors
    Nvec = zeros(3,size(feet, 2));
    for iFoot = 1:size(feet, 2)
        foot = feet(:,iFoot);
        Nvec(:,iFoot) = [-f(foot(1),foot(2),grid.dzdx,grid);...
                       -f(foot(1),foot(2),grid.dzdy,grid); 1];
        Nvec(:,iFoot) = Nvec(:,iFoot)/norm(Nvec(:,iFoot));
    end
    
    Fnorm = zeros(N,size(feet,2));
    Ftang = zeros(N,size(feet,2));
    cost = zeros(N,1);
    for n = 1:N
        fvec = x(n,2*Nq+1:end);
        F = reshape(fvec, 3, []);    
        for iFoot = 1:size(feet,2)
            Fnorm(n,iFoot) = -F(:,iFoot)'*Nvec(:,iFoot);
            Ftang(n,iFoot) = norm(cross(F(:,iFoot),Nvec(:,iFoot)));
            Fnorm(n,iFoot) = max(0, Fnorm(n,iFoot));
            cost(n) = gripperMargin(Fnorm(n,:), Ftang(n,:));
        end
    end
    c = max(cost);
    g = cost; % not actually the gradient
end

function [c,ceq] = constraints(x, r0, r1, iStep, grid, Nq, Nf, N, dt)
    q = x(:,1:Nq);
    dq = x(:,Nq+1:2*Nq);
    f = x(:,2*Nq+1:end);
    ddq = zeros(N,6);
    for n = 1:N
        ddq(n,:) = robotDynamics(q(n,:)', dq(n,:)', f(n,:)', r0.config, iStep);
    end
    c = [];
    % TODO: include all indices of q (full dynamics)
    ceq = constraintsDynamic(q(:,1:3), dq(:,1:3), ddq, f, r0.config, iStep, grid, dt);
%     ceq = [q(1,:)' - robot2state(r0); q(end,:)' - robot2state(r1); dq(1,:)'; dq(end,:)'; ceqd];
    for n = 1:N
        [ck, ceqk] = constraintsKinematic(q(n,:)', r0, iStep, grid);
        c = [c; ck];
        ceq = [ceq; ceqk];
    end
end

function [c,ceq] = constraintsKinematic(x, r0, iStep, grid)
    r = state2robot(x, r0.config);
    stanceFeet = r.vertices(:, r0.gait.feet(:,iStep) > 0);
    fixedFeet = r0.vertices(:, r0.gait.feet(:,iStep) > 0);
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

function ceq = constraintsDynamic(q, dq, ddq, fvec, config, iStep, grid, dt)
    torque = ddq(:,4:6);
    ddq = ddq(:,1:3);
    eom_dq = dq(2:end,:) - (dq(1:end-1,:) + dt*(ddq(2:end,:) + ddq(1:end-1,:))/2);
    eom_q = q(2:end,:) - (q(1:end-1,:) + dt*(dq(2:end,:) + dq(1:end-1,:))/2);
    ceq = [reshape(eom_dq, [], 1); reshape(eom_q, [], 1); reshape(torque, [], 1)];
end

function FRAMES = animateTrajectory(x, step, config, grid, Nq, Nf, N)
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    WEIGHT = 5*9.81; % Magnitude of gravity force (N)
    figure('units','normalized','outerposition',[0 0 .3 1]);
    FRAMES = struct('cdata',{},'colormap',{});
    for n = 1:N
        plotTerrain(grid);
        r = state2robot(x(n,1:Nq)', config);
        plotRobot(r);
        G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*WEIGHT;
        F = [reshape(x(n,2*Nq+1:end)', 3, []), G];
        plotForces(r, F, step-1, 'g', 0.016);
        drawnow();
        FRAMES(length(FRAMES)+1) = getframe(gcf);
    end
end

function [Fvec, T] = getForce(robot, count, grid, ddq)
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    MASS = 5; % Mass of robot (kg)
    g = 9.81; % Gravity (m/s^2)
    
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
    G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*g*MASS;
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
    beq = [-G+ddq(1:3)*MASS;zeros(3,1)];
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

function ddq = robotDynamics(q, dq, fvec, config, iStep)
    robot = state2robot(q, config);
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    MASS = 5; % Mass of robot (kg)
    g = 9.81; % Gravity (m/s^2)
    
    i = mod(iStep, size(robot.gait.angles, 2))+1;
    feet = robot.vertices(:, robot.gait.feet(:,i) > 0);
    r = feet - robot.origin;
    
    % Find contact forces
    G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*MASS*g;
    Aeq_torque = zeros(3,3*size(feet,2));
    for iFoot = 1:size(feet,2)
        ri = r(:,iFoot);
        Aeq_torque(:,iFoot*3-2:iFoot*3) = [0, ri(3), -ri(2);
                                           -ri(3), 0, ri(1);
                                           ri(2), -ri(1), 0];
    end
    Aeq = repmat(eye(3), 1, size(feet,2));
    beq = -G;
    ddq = (Aeq*fvec - beq)/MASS;
    torque = Aeq_torque*fvec;
    ddq = [ddq; torque];
end