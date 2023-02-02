% Four-legged robot with given leg DoF and either trot or quasistatic gait
function config = quadruped(dof, w, h, L, trot, horizon)
    config.count = [4, ones(1,dof(1)-1), 0, ones(1,dof(2)-1), 0,...
                       ones(1,dof(3)-1), 0, ones(1,dof(4)-1), 0];
    config.clearance = 0.04;
    config.bodies = {[-w/2,w/2,w/2,-w/2; h/2,h/2,-h/2,-h/2; 0,0,0,0]};
    config.iBodies = 0; % indices of joint for each body segment
    config.joints = zeros(3, 3, sum(config.count));
    config.limits = zeros(sum(config.count), 2); % angle limits
    config.gait.dyaw = deg2rad(5); % gait max yaw correction per step
    config.gait.horizon = horizon;
    if trot
        config.gait.angles = zeros(sum(config.count), 2); % gait states
        config.gait.feet = zeros(sum(config.count), 2); % foot contacts
        config.gait.dx = [0 0; 0.1 0.1; 0 0]; % gait centroid motion
    else
        config.gait.angles = zeros(sum(config.count), 4); % gait states
        config.gait.feet = zeros(sum(config.count), 4); % foot contacts
        config.gait.dx = 0.05*[0 0 0 0; 1 1 1 1; 0 0 0 0];
    end
    config.gait.buffer = zeros(size(config.gait.angles));
    x = [1;0;0]; % right
    y = [0;1;0]; % forward
    z = [0;0;1]; % up
    gait = [-1, 1, -1, 1]*40;
    feet = [1, 0, 1, 0];
    feet2 = [1 3 2 4];
    gait2 = [3/4, 1/4, -1/4, -3/4]*40;
    buffer = [15, 0, 0, 0];
    for iLeg = 1:4
        iJoint = sum(dof(1:iLeg-1))+1;
        config.joints(:,1,iJoint) = config.bodies{1}(:,iLeg); % shoulder
        config.joints(:,2,iJoint) = z; % shoulder yaw axis
        config.joints(:,2,iJoint+1) = y; % shoulder roll axis
        config.joints(:,3,iJoint+1) = x*L{dof(iLeg)-1}(1); % link 1
%         config.joints(:,3,iJoint+1) = x*L(1+(iLeg <= 2)); % link 1
        config.limits(iJoint:iJoint+1,:) = [-60-15, 60+15; -45, 90];
        config.gait.angles(iJoint+1,:) = 45;
        if trot
            config.gait.angles(iJoint,:) = [gait(iLeg), -gait(iLeg)];
            config.gait.feet(iJoint+1,:) = [feet(iLeg), ~feet(iLeg)];
        else
            config.gait.angles(iJoint, :) = circshift(gait2, feet2(iLeg)-1);
            config.gait.buffer(iJoint, :) = circshift(buffer, feet2(iLeg)-1);
            config.gait.feet(iJoint+1,:) = iLeg~=feet2;
        end
        if dof(iLeg) == 3
            config.joints(:,2,iJoint+2) = y; % elbow roll axis
            config.joints(:,3,iJoint+2) = x*L{dof(iLeg)-1}(2); % link 2
%             config.joints(:,3,iJoint+2) = x*L(1+(iLeg <= 2)); % link 2
            config.limits(iJoint+1, :) = [-90, 45];
            config.limits(iJoint+2, :) = [0, 135];
            config.gait.feet(iJoint+1,:) = 0;
            config.gait.angles(iJoint+1,:) = -30;
            config.gait.angles(iJoint+2,:) = 90;
            if trot
                config.gait.angles(iJoint,:) = [gait(iLeg), -gait(iLeg)];
                config.gait.feet(iJoint+2,:) = [feet(iLeg); ~feet(iLeg)];
            else
                config.gait.angles(iJoint, :) = circshift(gait2, feet2(iLeg)-1);
                config.gait.feet(iJoint+2,:) = iLeg~=feet2;
            end
        end
        config.gait.feet = logical(config.gait.feet);
        if iLeg == 1 || iLeg == 4 % mirror left legs
            config.joints(:,2:3,iJoint:end) = ...
                -config.joints(:,2:3,iJoint:end);
        end
    end
    config.parents = findParents(config.count);
    config.bodies{1} = interpolateBody(config.bodies{1}, 5);
end