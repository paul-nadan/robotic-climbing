"""
Control robot state using different feedback control strategies
"""
import numpy as np                  # pip install numpy
from qpsolvers import solve_qp      # pip install qpsolvers[quadprog]


def control_off(robot, *_args, **_kwargs):
    """
    Directly write setpoints to the motors
    """
    for motor in robot.motors.get():
        motor.set_angle = motor.goal_angle
        motor.set_velocity = motor.goal_velocity
        motor.set_torque = motor.goal_torque


def admittance_demo(robot, dt=0, init=False):
    """
    Demo controller for force on 1 foot in z-direction (set kp = ~5)
    """

    state = robot.state
    if init:
        state.last_err = 0
        return
    f = get_force_vector(robot)
    x = get_position_vector(robot, goal=False)
    x0 = get_position_vector(robot, goal=True)

    f_mat = np.hstack((np.reshape(f[:-1], (4, 3)).T, np.vstack((0, 0, f[-1]))))
    f = f_mat[:, 0]
    err = f - [0, 0, 3]
    x_err = (x0 - x)[0:3]
    d_err = (state.last_err - err)/dt
    state.last_err = err
    dx = state.kp * err + state.kd * d_err + state.kx * x_err
    print(np.array2string(f, precision=1, suppress_small=True),
          np.array2string(x_err, precision=1, suppress_small=True),
          np.array2string(dx, precision=1, suppress_small=True))
    robot.fl.move_in_direction(dt, *dx, np.linalg.norm(dx), direct=True)


def admittance(robot, dt=0, init=False):
    """
    Measure the force error and generate internal motions to apply feedback
    """

    # Initialize variables
    state = robot.state
    if init:
        state.prev_wrench = np.zeros((6,))                          # Reset previous error value
        state.integral_wrench = np.zeros((6,))                      # Reset error integral value
        return

    x = get_position_vector(robot, goal=False)                      # Current state vector
    x0 = get_position_vector(robot, goal=True)                      # Nominal state vector
    w = np.concatenate(([state.weights[0]] * 3, [state.weights[1]] * 3, [state.weights[2]] * 3, [state.weights[3]] * 3,
                        (state.weights[4],)))                       # Foot weight vector (13 x 1)
    G = (get_grasp_map(robot) * w)                                  # Weighted grasp map (6 x 13)
    f = get_force_vector(robot, goal=False)                         # Vector of forces applied by feet (13 x 1), 1 ms
    wrench = G @ f                                                  # Net wrench on body (6 x 1)
    G = G[:, :-1]                                                   # Remove tail for computing body displacement
    A = ((np.eye(13) * w)[:-1, :-1] - G.T @ np.linalg.pinv(G.T))    # Transformation to internal motion space (13 x 13)

    # Compute optimal force
    if not state.preload:
        f_goal = hang_simple(robot, wrench)                         # Optimal forces (3 x 5)
        state.f_goal = f_goal
    else:
        f_goal = hang_simple(robot, wrench)                         # Optimal forces (3 x 5)
        state.f_goal = f_goal

    # Compute feedback
    err = (mat2vec(f_goal) - f)[:-1]                                # Foot force error (12 x 1)
    err_pos = (x0 - x)[:-1]                                         # Foot position error (12 x 1)
    err_tail = (mat2vec(f_goal) - f)[-1]                            # Tail force error (1 x 1)
    dx = vec2mat(A @ (-state.kp * err + state.kx * err_pos))        # Foot displacement feedback (3 x 5)
    dx_tail = -state.kp * err_tail                                  # Tail displacement feedback (1 x 1)

    # Translate grippers and tail based on feedback
    for i, leg in enumerate(robot.get_legs()):
        leg.move_in_direction(dt, *dx[:, i], np.linalg.norm(dx[:, i]), direct=True)
    robot.tail.set_angle(robot.tail.motor.goal_angle + robot.tail.get_angvel(dx_tail * dt), direct=False)
    robot.tail.set_angle(robot.tail.motor.goal_angle, direct=True)


def get_grasp_map(robot):
    """
    Return the 6x13 grasp map between robot foot/tail velocity and body twist
    """
    g1 = np.hstack((np.identity(3), np.identity(3), np.identity(3), np.identity(3)))    # Feet linear (3 x 12)
    g1t = np.vstack((0, 0, 1))                                                          # Tail linear (3 x 1)
    g2 = np.hstack([skew(*leg.get_pos()) for leg in robot.get_legs()])                  # Feet angular (3 x 12)
    g2t = skew(*robot.tail.get_pos()) @ g1t                                             # Tail angular (3 x 1)

    return np.vstack((np.hstack((g1, g1t)), np.hstack((g2, g2t))))                      # Grasp map (6 x 13)


def get_position_vector(robot, goal=False):
    """
    Return the current robot foot positions as a vector: [foot 1, foot 2, foot 3, foot 4, tail z]
    """
    return np.concatenate((np.concatenate([leg.get_pos(goal=goal) for leg in robot.get_legs()]),
                           robot.tail.get_pos(goal=goal)[2:]))


def get_force_vector(robot, goal=False):
    """
    Return the current robot contact forces as a vector: [foot 1, foot 2, foot 3, foot 4, tail z]
    """
    return np.concatenate((np.concatenate([leg.get_force(goal=goal) for leg in robot.get_legs()]),
                           robot.tail.get_force(goal=goal)[2:]))


def skew(x, y, z):
    """
    Return a 3x3 skew-symmetric matrix for the given vector
    """
    return np.array(((0, -z, y), (z, 0, -x), (-y, x, 0)))


def hang_qp(robot, wrench, min_force=0, peel_angle=20):
    """
    Solve quadratic program to determine the optimal contact forces

    :param robot: Robot object
    :param wrench: Desired wrench on robot body
    :param min_force: Minimum gripper force in Newtons
    :param peel_angle: Maximum angle of applied force in degrees
    :return: 3x5 matrix of desired foot and tail forces
    """
    N = np.array(((0, 0, 0, 0), (0, 0, 0, 0), (1, 1, 1, 1)))

    H = np.eye(17)*0.1
    f = np.zeros(17)
    A = np.zeros((8, 17))
    b = np.zeros(8)
    Aeq = np.zeros((6, 17))
    beq = np.array(wrench)
    lb = np.zeros(17)
    ub = np.ones(17)*np.infty
    ub[[1, 5, 9, 13]] = 0.1
    rot = [np.eye(3) for _ in range(4)]
    r_hat = [np.eye(3) for _ in range(4)]
    best_cost = -1
    best_x = None
    best_mirror = None
    for i, leg in enumerate(robot.get_legs()):
        ind4 = slice(i*4, i*4+4)
        ind2 = slice(i*2, i*2+2)
        rot[i] = get_transform(N[:, i], (0*robot.state.gripper_angles[i])
                               * (-1 if leg.mirror else 1))
        r_hat[i] = skew(*leg.get_pos(goal=False))
        H[i*4:i*4+2, i*4:i*4+2] += 1
        H[i*4+3, i*4+3] += 1
        A[ind2, ind4] = -1*np.array(((1, -1, 0, 0), (tand(peel_angle), tand(peel_angle)/np.sqrt(2), 0, -1)))
        b[i*2] = -min_force
    Aeq[:3, -1] = robot.state.weights[-1] * np.array((0, 0, 1))
    Aeq[3:, -1] = robot.state.weights[-1] * skew(*robot.tail.get_pos(goal=False)) @ (0, 0, 1)

    for j in range(16):
        mirror = np.array([j//8 % 2, j//4 % 2, j//2 % 2, j % 2])*2-1
        for i in range(4):
            ind4 = slice(i * 4, i * 4 + 4)
            Aeq[:3, ind4] = robot.state.weights[i] * rot[i] @ np.array(((0, mirror[i], 0, 0),
                                                                        (1, 0, 0, 0), (0, 0, 1, -1)))
            Aeq[3:, ind4] = r_hat[i] @ Aeq[:3, ind4]
        x = solve_qp(H, f, A, b, Aeq, beq, lb, ub, solver="quadprog")
        if x is not None:
            cost = 0.5 * x.T @ H @ x
            if best_cost < 0 or cost < best_cost:
                best_cost = cost
                best_x = x
                best_mirror = mirror
    if best_x is None:
        return np.zeros((3, 5))
    forces = np.zeros((3, 5))
    for i in range(4):
        forces[:, i] = robot.state.weights[i] * rot[i] @ np.array(((0, best_mirror[i], 0, 0), (1, 0, 0, 0),
                                                                   (0, 0, 1, -1))) @ best_x[i * 4: i * 4 + 4]
    forces[:, -1] = (0, 0, best_x[-1])
    return forces


def hang_simple(robot, wrench, min_force=2):
    """
    Solve for optimal contact forces assuming ideal case (flat vertical wall)

    :param robot: Robot object
    :param wrench: Desired wrench on robot body
    :param min_force: Minimum gripper force in Newtons
    :return: 3x5 matrix of desired foot and tail forces
    """
    min_force = min(min_force, wrench[1]/4)
    r = robot.fl.origin[1]  # moment arm of feet
    L = robot.TAIL/r + 1  # moment arm of tail relative to moment arm of feet
    gravity = wrench[1] - 4 * min_force
    pitch = -wrench[3]

    stancePitch = -0.5*(L-1)/(L+1) + 0.5
    stanceN = np.array([-0.5, -0.5, 0, 0, 1])*pitch*stancePitch/r
    stanceT = np.array([0.5, 0.5, 0, 0, 0])*gravity

    swingPitch = 1/L
    swing1N = np.array([0, -0.5, -0.5, 0, 1])*pitch*swingPitch/r
    swing1T = np.array([0, 0.5, 0.5, 0, 0])*gravity
    swing2N = np.array([-0.5, 0, 0, -0.5, 1])*pitch*swingPitch/r
    swing2T = np.array([0.5, 0, 0, 0.5, 0])*gravity

    w1 = robot.state.weights[0]
    w2 = robot.state.weights[1]
    w3 = robot.state.weights[2]
    w4 = robot.state.weights[3]
    if w1 < 1:
        forceN = stanceN * w1 + swing1N * (1-w1)
        forceT = stanceT * w1 + swing1T * (1-w1)
        forceT[0] += min_force * w1
        forceT[2] += min_force * (2-w1)
        forceT[[1, 3]] += min_force
    elif w2 < 1:
        forceN = stanceN * w2 + swing2N * (1-w2)
        forceT = stanceT * w2 + swing2T * (1-w2)
        forceT[1] += min_force * w2
        forceT[3] += min_force * (2-w2)
        forceT[[0, 2]] += min_force
    else:
        forceN = stanceN
        forceT = stanceT
        if w3 < 1:
            forceT[2] += min_force * w3
            forceT[0] += min_force * (2-w3)
            forceT[[1, 3]] += min_force
        elif w4 < 1:
            forceT[3] += min_force * w4
            forceT[1] += min_force * (2-w4)
            forceT[[0, 2]] += min_force
        else:
            forceT[:4] += min_force

    forces = np.stack((np.zeros(5), forceT, forceN))
    # ratio = np.nanmax(-forces[2, :4]/forces[1, :4])
    return forces


def get_transform(normal, yaw):
    """
    Compute transform from gripper frame to robot body frame

    :param normal: Unit normal vector of surface
    :param yaw: Rotation about normal vector in deg (positive y-axis = 0, positive x-axis = 90)
    """
    r_yaw = np.array(((cosd(yaw), sind(yaw), 0), (-sind(yaw), cosd(yaw), 0), (0, 0, 1)))
    x = skew(0, 1, 0) @ normal
    y = skew(*normal) @ x
    r_normal = np.vstack((x, y, normal)).T
    return r_normal @ r_yaw


def vec2mat(x):
    """ Convert 13x1 vector of to 3x5 matrix """
    x = np.squeeze(x)
    return np.vstack((x[0:3], x[3:6], x[6:9], x[9:12], (0, 0, x[-1]))).T


def mat2vec(x_mat):
    """ Convert 3x5 matrix to 13x1 vector """
    return np.hstack((x_mat[:, 0], x_mat[:, 1], x_mat[:, 2], x_mat[:, 3], x_mat[2, -1])).T


def sind(a):
    return np.sin(np.deg2rad(a))


def cosd(a):
    return np.cos(np.deg2rad(a))


def tand(a):
    return np.tan(np.deg2rad(a))
