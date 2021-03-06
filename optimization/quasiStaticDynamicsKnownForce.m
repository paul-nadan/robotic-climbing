% Compute force components and torques from contact forces
function [F, Fnorm, Ftang, T, N] = quasiStaticDynamicsKnownForce(robot, count, F, grid)
    
    GRAVITY_ANGLE = 90; % Angle of gravity vector (vertical wall = 90)
    
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
    G = [0;-sind(GRAVITY_ANGLE);-cosd(GRAVITY_ANGLE)]*9.81;
%     G = [1;0;0]*9.81;
    Fvec = reshape(F, [], 1);
    F = [reshape(Fvec, 3, []), G];
    
    % Validate equilibrium
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
    equilibriumError = max(abs(Aeq*Fvec-beq))/max(abs(beq));
    if equilibriumError > 1e-5
        fprintf('Forces violate equilibrium constraint: error = %.3e\n', equilibriumError);
    end
    
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
        motor = robot.config.joints(:,2,iJoint);
        R = robot.R(:,:,iJoint);
        motor_world = R*motor;
        T(:, iJoint) = T(:, iJoint)'*motor_world;
        T(2:3, iJoint) = 0;
    end
end