% Translates or rotates a robot as a rigid body
function robot = moveRobot(robot, dx, dR)
    robot.links = dR*robot.links;
    robot.vertices = dR*(robot.vertices-robot.origin) + robot.origin;
    for iBody = 1:length(robot.bodies)
        robot.bodies{iBody} = dR*(robot.bodies{iBody}-robot.origin)+...
            robot.origin;
        robot.bodies{iBody} = robot.bodies{iBody} + dx;
    end
    robot.origin = robot.origin + dx;
    robot.vertices = robot.vertices + dx;
    robot.R0 = dR*robot.R0;
    for iR = 1:size(robot.R, 3)
        robot.R(:,:,iR) = dR*robot.R(:,:,iR);
    end
end