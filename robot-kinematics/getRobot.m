% Returns a robot with the given state and configuration
function robot = getRobot(origin, R0, angles, config)
    robot.origin = origin;
    robot.R0 = R0;
    robot.angles = angles;
    robot.config = config;
    robot.gait = config.gait;
    robot.R = zeros(3,3,length(angles));
    robot.links = zeros(3, length(angles));
    robot.vertices = zeros(3, length(angles));
    for iLink = 1:length(angles)
        robot.R(:,:,iLink) = vrrotvec2mat([config.joints(:,2,iLink); ...
            deg2rad(angles(iLink))]);
        if config.parents(iLink) == 0
            Rp = R0;
            Xp = origin;
        else
            Rp = robot.R(:,:,config.parents(iLink));
            Xp = robot.vertices(:, config.parents(iLink));
        end
        robot.R(:,:,iLink) = Rp*robot.R(:,:,iLink);
        robot.links(:,iLink) = robot.R(:,:,iLink)*config.joints(:,3,iLink);
        robot.vertices(:,iLink) = Xp + Rp*config.joints(:,1,iLink)+...
            robot.links(:,iLink);
    end
    robot.bodies = {};
    for i = 1:length(config.bodies)
        if config.iBodies(i)
            centroid = robot.vertices(:,config.iBodies(i));
            robot.bodies{i} = centroid + ...
                robot.R(:,:,config.iBodies(i))*config.bodies{i};
        else
            robot.bodies{i} = origin + ...
                R0*config.bodies{i};
        end
    end
    
    origins = [robot.origin, robot.vertices];
    Rall = cat(3, R0, robot.R);
    robot.com = origins(:,robot.config.iBodies+1);
    for iBody = 1:length(config.bodies)
        robot.com(:,iBody) = robot.com(:,iBody) +...
            Rall(:,:,robot.config.iBodies(iBody)+1)*robot.config.com(:,iBody);
    end
end