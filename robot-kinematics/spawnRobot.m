% Creates a new robot in an initial state
function robot = spawnRobot(origin, R0, config, grid, obstacleStep)
    i0 = size(config.gait.angles, 2);
    if obstacleStep ~= 0
        i0 = mod(obstacleStep-2, 4)+1;
    end
    robot = getRobot(origin, R0, config.gait.angles(:, i0), config);
    body = [robot.bodies{:}];
    dz = config.clearance + f(body(1, :), body(2, :), grid.z, grid) - body(3, :)+0.1;
    robot = moveRobot(robot, [0;0;max(dz)], eye(3));
    robot.config.threshold = 20;
    robot.seed = [];
    if ~obstacleStep
        robot = step2(robot, 0, grid, 0);
    end
    robot.fail = 0;
    robot.F = [];
    ymax = 0;
    feet = robot.vertices(:,any(robot.gait.feet,2));
    
%     ymax = feet(2,1);
    
    if obstacleStep == 1
        ymax = feet(2,2);
    elseif obstacleStep == 2
        ymax = feet(2,4);
    elseif obstacleStep == 3
        ymax = feet(2,2);
    elseif obstacleStep == 4
        ymax = feet(2,4);
    end
        
    robot = moveRobot(robot, [0;-ymax;0], eye(3));
end