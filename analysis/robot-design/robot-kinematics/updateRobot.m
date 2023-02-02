% Updates robot joint positions after a change in state variables
function robot = updateRobot(robot)
    robot = getRobot(robot.origin, robot.R0, robot.angles, robot.config);
end