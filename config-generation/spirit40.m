% Four-legged robot with given leg DoF and either trot or quasistatic gait
function config = spirit40(trot, horizon)
    w = 0.085*2;
    h = 0.225*2;
    j0 = [.07; .1857; 0];
    j1 = [.0763; .0463; 0];
    j2 = [.022; -0.2075; 0];
    j3 = [0; 0.2075; 0];
    
    config.count = [4, 1,1,0, 1,1,0, 1,1,0, 1,1,0];
    config.clearance = 0.04;
    config.bodies = {[-w/2,w/2,w/2,-w/2; h/2,h/2,-h/2,-h/2; 0,0,0,0]};
    signs = [-1,1,1,-1; 1,1,-1,-1; 0,0,0,0];
    config.iBodies = 0; % indices of joint for each body segment
    config.joints = zeros(3, 3, 12);
    config.limits = zeros(12, 2); % angle limits
    config.gait.dyaw = deg2rad(5); % gait max yaw correction per step
    config.gait.horizon = horizon;
    if trot
        config.gait.angles = zeros(12, 2); % gait states
        config.gait.feet = zeros(12, 2); % foot contacts
        config.gait.dx = [0 0; 0.1 0.1; 0 0]; % gait centroid motion
    else
        config.gait.angles = zeros(12, 4)-[0;45;-45;0;45;-45;0;45;-45;0;45;-45]; % gait states
        config.gait.feet = zeros(12, 4); % foot contacts
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
    for iLeg = 1:4
        iJoint = iLeg*3-2;
        config.joints(:,1,iJoint) = signs(:,iLeg).*j0; % shoulder
        config.joints(:,2,iJoint) = y; % shoulder roll forward axis
        config.joints(:,3,iJoint) = j1; % shoulder link
        
        config.joints(:,2,iJoint+1) = -signs(1,iLeg)*x; % shoulder pitch back axis
        config.joints(:,3,iJoint+1) = j2; % link 1
        
        config.joints(:,2,iJoint+2) = signs(1,iLeg)*x; % elbow roll axis
        config.joints(:,3,iJoint+2) = j3; % link 2
        
        config.limits(iJoint:iJoint+2,:) = rad2deg([-.43,.43; -inf, inf; -inf, inf]);
%         config.gait.angles(iJoint+1,:) = 45;
%         config.gait.feet(iJoint+1,:) = 0;
%         config.gait.angles(iJoint+1,:) = -30;
%         config.gait.angles(iJoint+2,:) = 90;
        if trot
            config.gait.angles(iJoint,:) = [gait(iLeg), -gait(iLeg)];
            config.gait.feet(iJoint+2,:) = [feet(iLeg); ~feet(iLeg)];
        else
            config.gait.angles(iJoint+2, :) = 45-circshift(gait2, feet2(iLeg)-1);
            config.gait.feet(iJoint+2,:) = iLeg~=feet2;
        end
        config.gait.feet = logical(config.gait.feet);
        if iLeg == 1 || iLeg == 4 % mirror left legs
            config.joints(1,2:3,iJoint:end) = ...
                -config.joints(1,2:3,iJoint:end);
        end
        if iLeg == 3 || iLeg == 4 % mirror back legs
            config.joints(2,3,iJoint) = ...
                -config.joints(2,3,iJoint);
        end
    end
    config.parents = findParents(config.count);
    config.bodies{1} = interpolateBody(config.bodies{1}, 5);
end