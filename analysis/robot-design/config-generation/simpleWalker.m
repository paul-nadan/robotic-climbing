% Code is binary values for: body joints (pitch/roll/yaw), knees joints 
% (front, middle, back), tail, second body joint, middle legs
% Leg order is FL FR ML MR BL BR
function config = simpleWalker(code, w, h, l, m, k)
    % Parse parameters
    if ~isnumeric(code)
        code = code == '1';
    end
    if ~code(9)
        code(5) = code(6);
        code(6) = 0;
    end
    if ~sum(code(1:3))>0
        code(8) = 0;
    end
    segs = 1+(sum(code(1:3))>0)*(1+code(8));
    dof = sum(code(1:3)).*(1+code(8)) + sum(code(4:6))*2 + code(7) + code(9)*4 + 8;
    w = w.*ones(segs,1);
    if length(h) == 1
        h = h.*ones(segs,1)/segs;
    end
    l = l.*ones(2+code(9)+code(7),1);
    if nargin <= 4
        m = 5;
    end
    if length(m) == 1
        m = m.*ones(segs,1)/segs;
    end
    if nargin <= 5
        k = 0.5;
    end
    k = k.*ones(2+code(9),1);
    
    % Define body segments
    config.bodies = cell(segs,1);
    config.iBodies = zeros(segs,1);
    config.com = zeros(3,segs);
    config.com(2,:) = -h/2;
    config.mass = m;
    for seg = 1:segs
        config.bodies{seg} = interpolateBody(getSegment(w(seg),h(seg)), 5);
        if seg == 2 % front legs + mid legs + front body joint
            config.iBodies(seg) = 2*(2+code(4)) + 2*(2+code(5))*code(9)*~code(8)+sum(code(1:3));
        elseif seg == 3 % front legs + mid legs + 2 body joints
            config.iBodies(seg) = 2*(2+code(4)) + 2*(2+code(5))*code(9)+2*sum(code(1:3));
        end
    end
    
    % Gait definition
    if code(9)
        period = 2;
        phases = [1,2,2,1,1,2];
    else
        period = 4;
        phases = [1,3,4,2];
    end
    config.gait.dx = l(1)/2/period*repmat([0;1;0], 1,period);
    
    % Initialization
    config.count = zeros(dof+1, 1);
    config.joints = zeros(3, 3, dof);
    config.limits = zeros(dof, 2);
    config.gait.feet = zeros(dof,period);
    config.gait.angles = zeros(dof,period);
    config.gait.buffer = zeros(dof,period);
    
    % Loop over legs
    j = 1; % current joint index
    seg = 1; % current body segment
    for leg = 1:2*(2+code(9))
        pair = floor((leg+1)/2);
        knee = code(pair+3);
        mirror = mod(leg,2);
        if pair == 1
            y = 0;
        elseif pair == 2 && code(9) && segs~=2
            y = -h(seg)/2;
        else
            y = -h(seg);
        end
        x = w(seg)/2;
        [joints, limits, angles, feet] = getLeg(x, y, l(pair), k(pair), knee, period, phases(leg), mirror);
        config.joints(:,:,j:j+1+knee) = joints;
        config.limits(j:j+1+knee,:) = limits;
        config.gait.angles(j:j+1+knee,:) = angles;
        config.gait.feet(j:j+1+knee,:) = feet;
        config.count(j+1:j+knee+1) = config.count(j+1:j+knee+1)+1;
        config.count(config.iBodies(seg)+1) = config.count(config.iBodies(seg)+1) + 1;
        j = j + 2 + knee;
        
        % Add body joint
        while 1
            iBody = find(j + sum(code(1:3))-1 == config.iBodies);
            if ~isempty(iBody)
                [joints, limits] = getBodyJoint(code(1:3), -h(iBody));
                iJoints = config.iBodies(seg+1)+1-sum(code(1:3)):config.iBodies(seg+1);
                parent = config.iBodies(seg)+1;
                config.joints(:,:,iJoints) = joints;
                config.limits(iJoints, :) = limits;
                j = j + sum(code(1:3));
                config.count(iJoints(1:end-1)+1) = config.count(iJoints(1:end-1)+1) + 1;
                config.count(parent) = config.count(parent)+1;
                seg = seg + 1;
            else
                break
            end
        end
    end
    
    % Add tail
    if code(7)
        [joints, limits] = getTail(-h(end), l(end));
        config.joints(:,:,end) = joints;
        config.limits(end,:) = limits;
        config.gait.angles(end,:) = 45;
        config.gait.feet(end,:) = 2;
        config.count(config.iBodies(end)+1) = config.count(config.iBodies(end)+1) + 1;
    end
    
    % Other properties
    config.parents = findParents(config.count);
    config.clearance = 0.04;
    config.gait.dyaw = deg2rad(5);
    config.gait.horizon = 1;%2;
    config.swivelType = 3; % Alignment: 1 = wrist, 2 = body, 3 = gravity
    config.swivel1 = deg2rad(0); % Change from starting angle during placement
    config.swivel2 = deg2rad(15); % Change in force direction after placement
    config.swivelOffset = [-1; 1; -1; 1]; % Offset angles
    if code(9)
        config.swivelOffset = [config.swivelOffset; config.swivelOffset(1:2)];
    end
%     config.swivelOffset = deg2rad(-90)*config.swivelOffset;
end

function body = getSegment(w, h)
    body = [-w/2,w/2,w/2,-w/2; -h,-h,0,0; 0,0,0,0];
end

function [joints, limits, angles, feet] = getLeg(x, y, l, k, knee, period, phase, mirror)
    joints = zeros(3,3,2+knee);
    limits = zeros(2+knee,2);
    angles = zeros(2+knee,period);
    feet = zeros(2+knee,period);
    joints(:,1,1) = [x;y;0]; % shoulder offset
    joints(:,2,1) = [0;0;1]; % shoulder yaw axis
    joints(:,3,1) = [1;0;0]*0.023; % yaw link (delete me)
    joints(:,2,2) = [0;1;0]; % shoulder roll axis
    joints(:,3,2) = [1;0;0]*l*max(~knee,k); % upper link
    limits(1,:) = [-75, 75];
    angles(1,:) = circshift(asind(linspace(0.5,-0.5,period)), phase-1);
    feet(2+knee,:) = 1;
    feet(2+knee,phase) = 0;
    if knee
        joints(:,2,3) = [0;1;0]; % elbow roll axis
        joints(:,3,3) = [1;0;0]*l*(1-k); % lower link
        limits(2,:) = [-90, 45];
        limits(3,:) = [0, 135];
        angles(2,:) = -30;
        angles(3,:) = 90;
    else
        angles(2,:) = 45;
        limits(2,:) = [-45, 90];
    end
    if mirror
        joints(:,2:3,:) = -joints(:,2:3,:);
        joints(1,1,1) = -joints(1,1,1);
    end
end

function [joints, limits] = getBodyJoint(pry, y)
    joints = zeros(3,3,sum(pry));
    limits = zeros(sum(pry),2);
    axes = find(pry);
    lims = [90, 45, 45];
    joints(:,1,1) = [0;y;0];
    for i = 1:sum(pry)
        joints(axes(i),2,i) = 1; % joint axis
        limits(i,:) = lims(axes(i))*[-1,1]; % joint limits
    end
end

function [joints, limits] = getTail(y, l)
    limits = [-45,90];
    joints = zeros(3,3,1);
    joints(:,1,1) = [0;y;0];
    joints(:,2,1) = [1;0;0];
    joints(:,3,1) = [0;-l;0];
end