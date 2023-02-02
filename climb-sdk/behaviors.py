"""
Carry out useful motions
"""

from controllers import *
from robot import cosd, sind

A1_SPRAWL = 45  # Swing leg maximum yaw angle in deg
A2_SPRAWL = (-45, -45, -45, -45)  # -45  # Sprawl shoulder angle in deg
A3_SPRAWL = (30, 30, 30, 30)  # 30  # Sprawl knee angle in deg
BODY_SPRAWL = 0  # Sprawl body angle in deg
TAIL_SPRAWL = -20  # Sprawl tail angle in deg
GAIT_ORDER = (3, 1, 4, 2)  # Footstep sequence for walking and climbing gaits

F_ENGAGE = (0, 4, 4)  # Horizontal, tangential, and normal components of engagement force in N
T_DEFAULT = 1400  # 1000  # Default torque limit
T_DEFAULT_YAW = 2000  # 1000  # Default torque limit
V_DEFAULT = 60  # Default velocity limit
T_SWING = 500  # Joint torque during leg swing in Nmm
T_DISENGAGE = 500  # Joint torque during gripper disengagement in Nmm
D_ENGAGE = (0, -40, -40)  # Foot translation vector during engagement in mm
D_DISENGAGE_1 = (0, 20, 0)  # Foot translation vector during disengagement in mm
D_DISENGAGE_2 = (0, 20, 20)  # Foot translation vector during disengagement in mm
D_DISENGAGE = (0, 40, 40)
D_PAW = 60  # Distance to travel before triggering pawing behavior
Z_PLACE = -160  # Minimum foot height for foot placement in mm
Z_LIFT = 15  # Maximum foot height for leg swing in mm
F_TOL = 1   # Force threshold tolerance


def stand(robot, _dt=0, init=False, term=False):
    """
    Stand upright
    """
    if term:
        return
    if init:
        robot.set_behavior(None)
    for leg in robot.legs.values():
        leg.set_angle(0, 30, -30)
    robot.body.set_angle(0)
    robot.tail.set_angle(-20)
    robot.state.step = 0
    robot.state.substep = 0


def sprawl(robot, _dt=0, init=False, term=False):
    """
    Adopt sprawled posture for climbing
    """
    if term:
        return
    if init:
        robot.set_behavior(None)
    yaws = [A1_SPRAWL, A1_SPRAWL/3, -A1_SPRAWL/3, -A1_SPRAWL]
    offsets = [2, 0, 3, 1]
    for i, leg in robot.legs.items():
        # leg.set_angle(yaws[(robot.state.step+offsets[i-1]) % 4], A2_SPRAWL, A3_SPRAWL)
        leg.set_angle(0*A1_SPRAWL/2, A2_SPRAWL[i-1], A3_SPRAWL[i-1])
        robot.state.gripper_angles[i-1] = 90  # robot.state.gripper_offsets[i-1] + leg.yaw_motor.goal_angle
    robot.body.set_angle(BODY_SPRAWL)
    robot.tail.set_angle(TAIL_SPRAWL)
    robot.state.step = 0
    robot.state.substep = 0
    robot.state.behavior_display = ""
    reset_torque(robot)
    reset_velocity(robot)


def reset_torque(robot):
    """
    Reset torque limits to default (e.g. after stick)
    """
    for motor in robot.motors.get():
        motor.goal_torque = T_DEFAULT
        if motor.model == "XM430-W350-T":
            motor.goal_torque = T_DEFAULT_YAW
        motor.set_torque = motor.goal_torque


def reset_velocity(robot):
    """
    Reset torque limits to default (e.g. after stick)
    """
    for motor in robot.motors.get():
        motor.goal_velocity = V_DEFAULT
        motor.set_velocity = motor.goal_velocity


def reset_angle(robot):
    """
    Reset motor setpoints to current value
    """
    for motor in robot.motors.get():
        motor.goal_angle = motor.angle
        motor.set_angle = motor.angle


def stick(robot, _dt=0, init=False, term=False):
    """
    Adopt sprawled posture for climbing
    """
    if term:
        reset_torque(robot)
        reset_angle(robot)
        return
    if init:
        sprawl(robot)
    for i, leg in robot.legs.items():
        leg.shoulder_motor.goal_torque = 200
        leg.knee_motor.goal_torque = 200
    robot.tail.goal_torque = 200
    robot.state.behavior_display = ""


def walk(robot, dt=0, init=False, term=False):
    """
    Simple upright walking gait
    """
    state = robot.state
    if term:
        return
    if init:
        state.t = 0
    state.t += dt
    if state.t < 1:
        return
    state.t = 0
    count = (state.step * 2 + state.substep) % 8
    yaw = [45, 45, 0, 0, -45, -45, -45, 45]
    shoulder = [30, 30, 30, 30, 30, 30, 0, 0]
    knee = [-30, -30, 0, 0, -30, -30, 0, 0]
    body = [0, 0, 0, 0, 0, 0, 0, 0]
    tail = [-20, -20, -20, -20, -20, -20, -20, -20]
    for leg in robot.legs:
        step = (count + 8 + GAIT_ORDER[leg - 1] * 2) % 8
        robot.legs[leg].set_angle(yaw[step], shoulder[step], knee[step])
    robot.body.set_angle(body[count])
    robot.tail.set_angle(tail[count])
    state.substep += 1
    if state.substep >= 2:
        state.substep = 0
        state.step += 1


def unload(robot, dt=0, init=False, term=False, leg=1, reload=True):
    state = robot.state
    if init:
        state.t = 0
    state.t += dt
    if reload:
        state.weights[leg-1] = min(1, state.t)
    else:
        state.weights[leg-1] = max(0, 1-state.t)


# def climb(robot, dt=0, init=False, term=False, teleop=True):
#     """
#     Sprawled climbing gait
#     """
#     state = robot.state
#     i_leg = GAIT_ORDER[state.step % 4]
#     leg = robot.legs[i_leg]
#     advance = False
#     if term:
#         state.weights = [1, 1, 1, 1, 1]
#         for leg in robot.legs.values():
#             leg.set_torque(1000, 1000, 1000)
#         state.substep = 0
#         return
#     if init:
#         state.substep, state.t, state.p = -1, 0, leg.get_pos(relative=True, goal=False)
#         state.x0, state.dy, state.z0 = robot.fl.get_pos(a1=A1_SPRAWL, a2=A2_SPRAWL, a3=A3_SPRAWL, relative=True)
#         state.teleop = False
#     state.t += dt
#
#     if state.teleop:
#         v = None
#         if state.teleop_key == "w":
#             leg.move_in_direction(dt, 0, 1, 0, v, relative=False)
#         elif state.teleop_key == "a":
#             leg.move_in_direction(dt, -1, 0, 0, v, relative=False)
#         elif state.teleop_key == "s":
#             leg.move_in_direction(dt, 0, -1, 0, v, relative=False)
#         elif state.teleop_key == "d":
#             leg.move_in_direction(dt, 1, 0, 0, v, relative=False)
#         elif state.teleop_key == "q":
#             leg.move_in_direction(dt, 0, 0, 1, v, relative=False)
#         elif state.teleop_key == "e":
#             leg.move_in_direction(dt, 0, 0, -1, v, relative=False)
#         elif state.teleop_key == " ":
#             state.teleop = False
#             state.t = 0
#         state.teleop_key = None
#     elif state.substep == -1:  # Recenter body
#         if robot.move_to_position(dt, y=0):
#             advance = True
#     elif state.substep == 0:  # Unload foot
#         leg.set_torque(T_DISENGAGE, T_DISENGAGE, T_DISENGAGE)
#         state.weights[GAIT_ORDER[state.step % 4] - 1] = max(1 - state.t, 0)
#         if state.t > 1:
#             advance = True
#     elif state.substep == 1:  # Disengage foot (limit torque, move foot up)
#         a = state.gripper_angles[i_leg - 1]
#         d1 = (D_DISENGAGE_1[0] * sind(a) + D_DISENGAGE_1[1] * cosd(a),
#               -D_DISENGAGE_1[0] * cosd(a) + D_DISENGAGE_1[1] * sind(a),
#               D_DISENGAGE_1[2]
#               )
#         if leg.move_to_position(dt, *(state.p + np.array(d1)), relative=True):
#             advance = True
#     elif state.substep == 2:  # Disengage foot (limit torque, move foot up and away from wall)
#         a = state.gripper_angles[i_leg - 1]
#         d2 = (D_DISENGAGE_2[0] * sind(a) + D_DISENGAGE_2[1] * cosd(a),
#               -D_DISENGAGE_2[0] * cosd(a) + D_DISENGAGE_2[1] * sind(a),
#               D_DISENGAGE_2[2]
#               )
#         if leg.move_to_position(dt, *(state.p + np.array(d2)), relative=True):
#             advance = True
#     elif state.substep == 3:  # Lift foot (move foot vertically)
#         if leg.move_to_position(dt, z=Z_LIFT, relative=True):
#             advance = True
#     elif state.substep == 4:  # Swing foot (limit torque, move foot in plane, move body)
#         leg.set_torque(T_SWING, T_SWING, T_SWING)
#         if leg.move_to_position(dt, x=state.x0, y=state.dy, relative=True):
#             advance = True
#             state.teleop = teleop
#         robot.move_to_position(dt, y=0)
#     elif state.substep == 5:  # Place foot (limit torque, move foot vertically)
#         # leg.set_torque(100, 100, 100)
#         # recenter(robot, dt)
#         if leg.move_to_position(dt, z=Z_PLACE, relative=True):
#             state.gripper_angles[i_leg - 1] = state.gripper_offsets[i_leg - 1] + leg.yaw_motor.goal_angle
#             advance = True
#     elif state.substep == 6:  # Engage foot (limit force, move foot down and into wall)
#         leg.set_force(*F_ENGAGE, relative=True)
#         a = state.gripper_angles[i_leg - 1]
#         d = (D_ENGAGE[0] * sind(a) + D_ENGAGE[1] * cosd(a),
#              -D_ENGAGE[0] * cosd(a) + D_ENGAGE[1] * sind(a),
#              D_ENGAGE[2])
#         if leg.move_to_position(dt, *(state.p + np.array(d)), relative=True):
#             advance = True
#     elif state.substep == 7:  # Load foot
#         state.weights[GAIT_ORDER[state.step % 4] - 1] = min(state.t, 1)
#         if state.t > 1:
#             leg.set_torque(1000, 1000, 1000)
#             advance = True
#     elif state.substep == 8:  # Recenter body
#         recenter(robot, dt)
#         if state.t > 1:
#             advance = True
#     else:  # Terminate
#         state.step += 1
#         state.substep = 0
#         robot.set_behavior(None)
#         return
#
#     if advance:
#         state.substep, state.t, state.p = state.substep + 1, 0, leg.get_pos(relative=True, goal=False)
#     substeps = ["Unload", "Disengage 1", "Disengage 2", "Lift", "Swing", "Place", "Engage", "Load", "Recenter",
#                 "Completed"]
#     state.behavior_display = f"Substep: {'Teleop' if state.teleop else substeps[state.substep]}"


def climb2(robot, dt=0, init=False, term=False, teleop=False):
    """
    Sprawled climbing gait for admittance controller
    """
    state = robot.state
    i_leg = GAIT_ORDER[state.step % 4]
    leg = robot.legs[i_leg]
    advance = False
    if term:
        state.weights = [1, 1, 1, 1, 1]
        for leg in robot.legs.values():
            leg.set_torque(1000, 1000, 1000)
        state.substep = 0
        return
    if init:
        state.substep, state.t, state.p, state.paw = 0, 0, leg.get_pos(relative=True, goal=False), False
        state.x0, state.dy, state.z0 = robot.fl.get_pos(a1=A1_SPRAWL, a2=A2_SPRAWL[0], a3=A3_SPRAWL[0], relative=True)
        state.teleop = False
        state.disengage_step = 0
    state.t += dt

    if state.teleop:
        v = None
        if state.teleop_key == "w":
            leg.move_in_direction(dt, 0, 1, 0, v, relative=False)
        elif state.teleop_key == "a":
            leg.move_in_direction(dt, -1, 0, 0, v, relative=False)
        elif state.teleop_key == "s":
            leg.move_in_direction(dt, 0, -1, 0, v, relative=False)
        elif state.teleop_key == "d":
            leg.move_in_direction(dt, 1, 0, 0, v, relative=False)
        elif state.teleop_key == "q":
            leg.move_in_direction(dt, 0, 0, 1, v, relative=False)
        elif state.teleop_key == "e":
            leg.move_in_direction(dt, 0, 0, -1, v, relative=False)
        elif state.teleop_key == " ":
            state.teleop = False
            state.t = 0
        state.teleop_key = None
    elif state.substep == 0:  # Set foot weight to zero, lower goal force to disengagement limit
        scale = (1 - state.t) + T_DISENGAGE * state.t
        leg.set_torque(scale*T_DEFAULT_YAW, scale*T_DEFAULT, scale*T_DEFAULT)
        state.weights[GAIT_ORDER[state.step % 4] - 1] = 0
        leg.set_angle(*leg.get_angle())
        leg.set_angle(*leg.get_angle(), direct=True)
        if state.t > 1:
            advance = True
            state.disengage_step = 0
    # elif state.substep == 1:  # Raise foot while Fz > 0 and disengage foot while Fy > 0
    #     Fx, Fy, Fz = leg.get_force(relative=True)
    #     a = state.gripper_angles[i_leg - 1]
    #     d = [D_DISENGAGE[0] * sind(a) + D_DISENGAGE[1] * cosd(a),
    #          -D_DISENGAGE[0] * cosd(a) + D_DISENGAGE[1] * sind(a),
    #          D_DISENGAGE[2]]
    #     if Fx * cosd(a) + Fy * sind(a) < -F_TOL:
    #         d[0] *= -1
    #         d[1] *= -1
    #     if Fz < -F_TOL:
    #         d[2] *= -1
    #     print(f'{Fx:.1f}, {Fy:.1f}, {Fz:.1f} | {d[0]:.1f}, {d[1]:.1f}, {d[2]:.1f}')
    #     if leg.move_to_position(dt, *(state.p + np.array(d)), relative=True, v=50):
    #         advance = True
    elif state.substep == 1:  # Raise foot while Fz > 0 and disengage foot while Fy > 0
        Fx, Fy, Fz = leg.get_force(relative=True)
        a = 90  # state.gripper_angles[i_leg - 1]
        d = [D_DISENGAGE[0] * sind(a) + D_DISENGAGE[1] * cosd(a),
             -D_DISENGAGE[0] * cosd(a) + D_DISENGAGE[1] * sind(a),
             D_DISENGAGE[2]]
        Ft = Fx * cosd(a) + Fy * sind(a)
        if state.disengage_step == 0:  # Move upward to disengage spines
            d[2] *= 0
            if Ft < F_TOL:  # No load on the gripper
                state.disengage_step += 1
        elif state.disengage_step == 1:  # Raise foot
            d[0] *= 0
            d[1] *= 0
            if Fz < -3:  # Stuck
                state.disengage_step += 1
        elif state.disengage_step == 2:  # Move upward to disengage spines
            d[2] *= 0
            if Ft < F_TOL:
                state.disengage_step += 1
        else:  # Press into surface
            d[0] *= 0
            d[1] *= 0
            d[2] *= -1
            if Fz > 0:  # Pressed into surface
                state.disengage_step = 0
        # print(f'{Fx:.1f}, {Fy:.1f}, {Fz:.1f} | {d[0]:.1f}, {d[1]:.1f}, {d[2]:.1f}')
        # print(state.disengage_step, leg.shoulder_motor.set_angle, d)
        if leg.move_in_direction(dt, *np.array(d), relative=True, v=500):
            advance = True
        # if leg.shoulder_motor.set_angle <= leg.shoulder_motor.lower+1:
        #     advance = True
        # if leg.get_pos()[2] >= Z_LIFT:  # Clear of surface
        #     advance = Truee
    elif state.substep == 2:  # Lift foot (move foot vertically)
        # if leg.move_to_position(dt, z=Z_LIFT, relative=True):
        #     advance = True
        advance = True
    elif state.substep == 3:  # Swing foot (limit torque, move foot in plane, move body)
        leg.set_torque(T_SWING, T_SWING, T_SWING)
        if leg.move_to_position(dt, x=state.x0, y=state.dy, relative=True):  # and state.t > 3:
            advance = True
            if not state.paw:
                state.teleop = teleop
        Fx, Fy, Fz = leg.get_force(relative=True)
        if Fy < -4:  # Gripper is caught on surface
            state.substep = 1
            state.paw = True
            advance = True
            print("------PAW------")
            leg.set_angle(*leg.get_angle(), direct=True)
            leg.set_angle(*leg.get_angle(), direct=False)
        # robot.move_to_position(dt, y=0)
        recenter(robot, dt, offset=1, swing=i_leg)
        # robot.move_to_position(dt, y=0, v=30)
    elif state.substep == 4:  # Engage foot (limit force, move foot down and into wall)
        leg.set_torque(T_DEFAULT_YAW, T_DEFAULT, T_DEFAULT)
        Fx, Fy, Fz = leg.get_force(relative=True)
        a = 90  # state.gripper_angles[i_leg - 1]
        d = [D_ENGAGE[0] * sind(a) + D_ENGAGE[1] * cosd(a),
             -D_ENGAGE[0] * cosd(a) + D_ENGAGE[1] * sind(a),
             D_ENGAGE[2]]
        if Fz < F_ENGAGE[2] - F_TOL:  # Lower foot, do not start loading yet
            d[0] = 0
            d[1] = 0
        elif Fz > F_ENGAGE[2] + F_TOL:  # Raise foot while loading
            d[2] *= -1
        else:  # Maintain foot height while loading
            d[2] *= 0
        # print(f'{Fx:.1f}, {Fy:.1f}, {Fz:.1f} | {d[0]:.1f}, {d[1]:.1f}, {d[2]:.1f}')
        leg.move_in_direction(dt, *d, relative=True, v=50)
        x, y, z = leg.get_pos(relative=True)
        # print((state.p[0], state.p[1]), (x, y), (state.p[1] - y) * sind(a) + (state.p[0] - x) * cosd(a))
        if (state.p[1] - y) * sind(a) + (state.p[0] - x) * cosd(a) > D_PAW:
            state.substep = 1
            state.paw = True
            advance = True
            print("------PAW------")
            leg.set_angle(*leg.get_angle(), direct=True)
            leg.set_angle(*leg.get_angle(), direct=False)
        elif Fx * cosd(a) + Fy * sind(a) > F_ENGAGE[1] + F_TOL:
            state.paw = False
            advance = True
            state.weights[GAIT_ORDER[state.step % 4] - 1] = 1
            leg.set_angle(*leg.get_angle(), direct=True)
            leg.set_angle(*leg.get_angle(), direct=False)
            state.gripper_angles[i_leg - 1] = 90  # state.gripper_offsets[i_leg - 1] + leg.yaw_motor.goal_angle
    elif state.substep == 5:  # Recenter body
        recenter(robot, dt, offset=1)
        if state.t > 1:
            advance = True
        # TODO: Paw if tangential force on swing foot is below threshold (2N?)
    else:  # Terminate
        state.step += 1
        state.substep = 0
        robot.set_behavior(climb2)
        return

    if robot.controller == control_off:
        robot.tail.set_angle(robot.tail.motor.angle-2)
        robot.tail.set_torque(200)
    if advance:
        state.substep, state.t, state.p = state.substep + 1, 0, leg.get_pos(relative=True, goal=False)
    substeps = ["Unload", "Disengage", "Lift", "Swing", "Engage", "Recenter",
                "Completed"]
    state.behavior_display = f"Substep: {'Teleop' if state.teleop else substeps[state.substep]}"


def recenter(robot, dt=0, init=False, term=False, offset=0, swing=0):
    """
    Align the robot body to minimize twist from the standard sprawled posture
    """
    speed = 60
    if init or term:
        return
    G_tail = get_grasp_map(robot)
    G = G_tail[:, :12]
    x = get_position_vector(robot, False)[:12]
    x0 = get_stance(robot, offset)[:12]
    dx = x0 - x
    if swing:
        # dx[swing*3-3] = 0  # ignore swing x
        dx[swing*3-1] = 0  # ignore swing z
    y = G_tail.T @ np.diag([1, 1, 1, 1, 1, 0]) @ np.linalg.pinv(G).T @ dx
    # print(y)
    # y += G_tail.T @ np.array([0, 0, 0, 0, 0, .1])
    # G_yaw = G_tail[-1:, :]
    # y = (np.eye(13) - G_yaw.T @ np.linalg.pinv(G_yaw.T)) @ y
    d_max = 0
    for i in range(4):
        d_max = max(d_max, np.linalg.norm(y[3 * i:3 * i + 3]))
    d_max = max(d_max, 8*speed*dt)
    for i in range(4):
        d = np.linalg.norm(y[3 * i:3 * i + 3])
        v = speed * d / d_max
        robot.legs[i+1].move_in_direction(dt, *y[3 * i:3 * i + 3], v)
        # print(y[3 * i:3 * i + 3], v)
    v_tail = min(speed * abs(y[12]) / d_max, abs(y[12]) * 1000 / dt)
    robot.tail.set_pos(robot.tail.get_pos(goal=True)[2] + dt * v_tail * np.sign(y[12]))
    # print(speed/d_max, 1/dt)


def get_stance(robot, offset=0):
    """
    Get the robot's sprawled posture at the current step index
    """
    x = np.zeros(13)
    yaws = [A1_SPRAWL, A1_SPRAWL/3, -A1_SPRAWL/3, -A1_SPRAWL]
    offsets = [2, 0, 3, 1]
    # print(robot.state.step+offset)
    for i in range(4):
        if robot.state.step + offset:
            yaw = 2/3*yaws[(robot.state.step+offset+offsets[i]) % 4]
            # print(i+1, yaw)
        else:
            yaw = 0
        x[i*3:i*3+3] = robot.legs[i + 1].get_pos(yaw, A2_SPRAWL[i], A3_SPRAWL[i])
    x[-1] = robot.tail.get_pos(TAIL_SPRAWL)[2]
    return x


def test_grasp(robot, dt=0, init=False, term=False, foot=2, pitch=30, yaw=0, ft=5, fn=4, fmax=15, n=10):
    """
    Attempt a grasp with specified leg and record the force at failure
    """
    state = robot.state
    leg = robot.legs[foot]
    if init:
        state.substep, state.f, state.n, state.fail = 0, (0, 0, 0), n, False
        state.x0, state.y0, state.z0 = robot.fl.get_pos(a1=A1_SPRAWL, a2=A2_SPRAWL[0], a3=A3_SPRAWL[0], relative=True)
        sprawl(robot)
        for other_leg in robot.legs.values():
            if other_leg != leg:
                other_leg.set_torque(0, 0, 0)
        leg.set_angle(a1=A1_SPRAWL)
        leg.set_torque(3000, 1800, 1800)
    if term:
        reset_angle(robot)
        for other_leg in robot.legs.values():
            other_leg.set_torque(1000, 1000, 1000)
        return
    state.t += dt
    advance = False
    fx, fy, fz = leg.get_force(relative=True)
    x, y, z = leg.get_pos(relative=True)

    if state.substep == 0:  # Raise foot
        if leg.move_to_position(dt, x=state.x0, y=state.y0, z=state.z0, relative=True):
            advance = True
    elif state.substep == 1:  # Lower foot
        leg.move_in_direction(dt, 0, 0, -1, relative=True, v=30)
        if fz > fn:
            advance = True
    elif state.substep == 2:  # Engage foot
        leg.move_in_direction(dt, 0, -1, 0, relative=True, v=30)
        if fy > ft:
            advance = True
        if y - state.y0 < -100:  # Failed to engage
            state.substep = 0
            return
    elif state.substep == 3:  # Load foot
        oob = leg.move_in_direction(dt, 0, -cosd(pitch), sind(pitch), relative=True, v=30)
        # if fy > state.f[1]:
        if fz < state.f[2]:
            state.f = (fx, fy, fz)
        if oob or fy > fmax or fy < ft/2:
            state.fail = fy < ft
            advance = True
            print(f'[{state.f[0]}, {state.f[1]}, {state.f[2]}, {state.fail * 1}]')
        # print(oob, fy, y - state.y0)
    elif state.substep == 4:  # Unload foot
        reset_angle(robot)
        if leg.move_to_position(dt, x=state.x0, y=state.y0, relative=True):
            advance = True
    else:
        if state.n <= 1:
            robot.set_behavior(None)
        else:
            state.n -= 1
            state.substep = 0
            state.f = (0, 0, 0)
    if advance:
        state.substep += 1
