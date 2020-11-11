% Six-legged robot geometry with 3-DoF per leg and 2 body joints
function config = hexapod(w, h, L, horizon)
    leg = [1 1 0];
    config.count = [3, 3, 2, leg, leg, leg, leg, leg, leg];
    config.clearance = 0.0;
    body = [-w/2,w/2,w/2,-w/2; h,h,0,0; 0,0,0,0];
    config.bodies = {body, body, body};
    config.iBodies = 0:2; % indices of joint for each body segment
    config.joints = zeros(3, 3, sum(config.count));
    config.limits = zeros(sum(config.count), 2); % angle limits
    config.limits(1:2,:) = 90*[-1, 1; -1, 1]; % angle limits
    config.gait.angles = zeros(sum(config.count), 2); % gait states
    config.gait.feet = zeros(sum(config.count), 2); % gait foot contacts
    config.gait.dx = [0 0; 0.1 0.1; 0 0]; % gait centroid motion
    config.gait.dyaw = deg2rad(5); % gait max yaw correction per step
    config.gait.horizon = horizon;
    x = [1;0;0]; % right
    y = [0;1;0]; % forward
    z = [0;0;1]; % up
    gait = [-1, 1, -1, 1, -1, 1, -1, 1]*40;
    feet = [1, 0, 1, 0];
    config.joints(:,1,1:2) = [[0;h;0],[0;h;0]];
    config.joints(:,2,1:2) = [x,x];
    for iBody = 1:length(config.iBodies)
        for iLeg = 1:2
            iJoint = iLeg*3 + (iBody-1)*6;
            config.joints(1,1,iJoint) = config.bodies{iBody}(1,iLeg); % shoulder
            config.joints(2,1,iJoint) = h/2; % shoulder
            config.joints(:,2,iJoint) = z; % shoulder yaw axis
            config.joints(:,2,iJoint+1) = y; % shoulder roll axis
            config.joints(:,3,iJoint+1) = x*L; % link 1
            config.limits(iJoint:iJoint+1,:) = [-60, 60; -60, 60];
            config.gait.angles(iJoint,:) = [gait(iLeg), -gait(iLeg)];
            config.gait.feet(iJoint+1,:) = [feet(iLeg), ~feet(iLeg)];
            config.joints(:,2,iJoint+2) = y; % elbow roll axis
            config.joints(:,3,iJoint+2) = x*L; % link 2
            config.limits(iJoint+1, :) = [-45, 0];
            config.limits(iJoint+2, :) = [0, 135];
            config.gait.feet(iJoint+1,:) = [0, 0];
            config.gait.feet(iJoint+2,:) = [feet(iLeg); ~feet(iLeg)];
            config.gait.angles(iJoint+1:iJoint+2,:) = [-45, -45; 135, 135];
            config.gait.feet = logical(config.gait.feet);
            if iLeg == 1 || iLeg == 4 % mirror left legs
                config.joints(:,2:3,iJoint:end) = ...
                    -config.joints(:,2:3,iJoint:end);
            end
            if iBody == 2
                config.gait.angles(iJoint,:) = -config.gait.angles(iJoint,:);
                config.gait.feet(iJoint+2,:) = ~config.gait.feet(iJoint+2,:);
            end
        end
    end
    config.parents = findParents(config.count);
    config.bodies{1} = interpolateBody(config.bodies{1}, 5);
    config.bodies{2} = interpolateBody(config.bodies{1}, 5);
    config.bodies{3} = interpolateBody(config.bodies{1}, 5);
end