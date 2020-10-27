% Creates a new robot in an initial state
function robot = spawnRobot(origin, R0, config, grid)
    robot = getRobot(origin, R0, config.gait.angles(:, end), config);
    body = [robot.bodies{:}];
    dz = config.clearance + f(body(1, :), body(2, :), grid.z, grid) - body(3, :)+0.1;
    robot = moveRobot(robot, [0;0;max(dz)], eye(3));
    robot = step(robot, 0, grid);
end