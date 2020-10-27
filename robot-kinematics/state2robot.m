% Creates a robot with the given state vector and configuration
function robot = state2robot(state, config)
    B = cross(state(4:6), state(7:9));
    R0 = [-state(7:9), state(4:6), B];
    robot = getRobot(state(1:3), R0, state(10:end), config);
end