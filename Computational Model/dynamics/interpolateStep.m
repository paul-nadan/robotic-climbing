% t = 0:5;
% q = t.^3;
% dq = 3*t.^2
% ddq = 6*t
% [dq1, ddq1] = computeDerivatives(dq(1), q, t)
% return
config = quadruped([3,3,2,2], 0.1, 0.3, {.2, [.16, .16]}, 0, 2);
seed = 42;
i = 1;
global ANIMATE RECORD
ANIMATE = 1;
RECORD = 0;
% grid = terrain([-.75, 1.25], ...
%         [-1.5 0.5], .01, [1,1,0.5], [1, .25, 0.0625], 0, [0;-.75;0], seed);
% robot0 = spawnRobot(grid.spawn, eye(3), config, grid);
% robot1 = step(robot0, i, grid);

iters = 5;

% robots = repmat(robot0, 2^iters+1, 1);
% q0 = robot2state(robot0);
% q = zeros(length(q0), length(robots));
% q(:,1) = q0;
% q(:,end) = robot2state(robot1);
% robots(end) = robot1;
% for iIter = 1:iters
%     for iRobot = 1:2^(iIter-1)
%         tic()
%         offset = 2^(iters-iIter);
%         index = 1+offset+2^(1+iters-iIter)*(iRobot-1);
%         [robots(index), q(:,index)] = interpolate(robots(index-offset), robots(index+offset), i, grid);
%     end
% end
tmax = 1;
t = linspace(0,tmax,size(q, 2));
[dq, ddq] = computeDerivatives(zeros(size(q, 1),1), q, t);
% F = zeros(9, size(q,2));
% for it = 1:size(q,2)
%     F(:,it) = getForce(robots(it), i, grid, ddq(:,it));
% end
% F

% TIME_STEP = 0.25;
% for iRobot = 1:length(robots)-1
%     animateStep(robots(iRobot), robots(iRobot+1), TIME_STEP, i, grid);
% end
% plotRobot(robots(end));
figure('units','normalized','outerposition',[0 0 .3 1]);
FRAMES = struct('cdata',{},'colormap',{});
for iRobot = 1:length(robots)
    plotTerrain(grid);
    plotRobot(robots(iRobot));
    G = [0;0;0];
    Freshaped = [reshape(F(:,iRobot), 3, []), G];
    plotForces(robots(iRobot), Freshaped, i-1, 'g', 0.016);
    drawnow();
    FRAMES(length(FRAMES)+1) = getframe(gcf);
end

function [r, x] = interpolate(r0, r1, i, grid)
    x0 = robot2state(r0);
    x1 = robot2state(r1);
    xbar = (x0+x1)/2;
    lb = min(x0, x1);
    ub = max(x0, x1);
    options = optimoptions('fmincon','MaxFunctionEvaluations',1e4,...
        'Algorithm','sqp','ConstraintTolerance',1e-4,...
        'Display','off','SpecifyObjectiveGradient',true, 'CheckGradients', false, 'FiniteDifferenceType', 'central');
    [x,~,~,output] = fmincon(@(x)cost(x,xbar),xbar,[],[],...
        [],[],lb,ub,@(x)constraints(x,r0,i,grid),options);
    r = state2robot(x, r0.config);
    r.fail = output.constrviolation > options.ConstraintTolerance;
    if r.fail
        disp('Failed!');
    end
    r.F = 0;
    r.seed = 0;
end

function [c, g] = cost(x, xbar)
    e = x-xbar;
    c = sum(e.^2);
    g = 2*e;
end

function [c,ceq] = constraints(x, r0, i, grid)
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

function [dq, ddq] = computeDerivatives(dq0, q, t)
%     dq = zeros(size(q));
%     ddq = zeros(size(q));
%     dq(:,1) = dq0;
%     for i = 2:size(q, 2)
%         dt = t(i) - t(i-1);
%         ddq(:,i) = 2*(q(:,i) - q(:,i-1) - dq(:,i-1)*dt)/dt^2;
%         dq(:,i) = dq(:,i-1) + ddq(:,i)*dt;
%     end
%     dq = diff(q,2)./diff(t);
    dq = [dq0,diff(q,1,2)./diff(t,1)];
    ddq = [diff(dq,1,2)./diff(t,1), dq*0];
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