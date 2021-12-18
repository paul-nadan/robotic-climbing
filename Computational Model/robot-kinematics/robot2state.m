% Returns the state vector describing a given robot
function state = robot2state(robot)
    T0 = robot.R0(:,2);
    N0 = -robot.R0(:,1);
    state = [robot.origin; T0; N0; robot.angles];
end