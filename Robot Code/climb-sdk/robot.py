"""
Evaluate current robot state and modify setpoints using forward and inverse kinematics
"""
from types import SimpleNamespace

from controllers import *

# Motor limits (deg)
YAW_LIMIT_MIN = -70
YAW_LIMIT_MAX = 70
SHOULDER_LIMIT_MIN = -60
SHOULDER_LIMIT_MAX = 90
KNEE_LIMIT_MIN = -60  # -90
KNEE_LIMIT_MAX = 60
BODY_LIMIT_MIN = -60
BODY_LIMIT_MAX = 60
TAIL_LIMIT_MIN = -80
TAIL_LIMIT_MAX = 80

# Robot geometry (mm)
WIDTH = 154  # Body width
LENGTH = 283.4  # Body length
HEIGHT = 54  # Body height
LINK1 = 23.5  # Shoulder link length
LINK2A = 111  # Upper leg link length
LINK2B = 22.5  # Upper leg link vertical offset
LINK3 = 98.7  # Lower leg link length
TAIL = 330.0  # Tail length

# Leg numbering
FL = 1  # Front left
FR = 2  # Front right
RL = 3  # Rear left
RR = 4  # Rear right

# Controller defaults
KP = 10      # Proportional term (N/mm)
KI = 0      # Integral term (N/mm-s)
KD = 0      # Derivative term (N-s/mm)
KR = 1000   # Relative feedback strength for angular error
KO = 0.1    # Angle overshoot to generate torque
KX = 0      # Position kP term for admittance control / Inward grasping force (N)
KM = 30     # Expected robot weight (N)


class Robot:
    def __init__(self, motors, controller=control_off):
        yaw = motors.add([1, 2, 3, 4], 'XM430-W350-T', lower=YAW_LIMIT_MIN, upper=YAW_LIMIT_MAX)
        shoulder = motors.add([5, 6, 7, 8], 'AX18-A', lower=SHOULDER_LIMIT_MIN, upper=SHOULDER_LIMIT_MAX, mirror=(6, 7))
        knee = motors.add([9, 10, 11, 12], 'AX18-A', lower=KNEE_LIMIT_MIN, upper=KNEE_LIMIT_MAX, mirror=(10, 11),
                          offset={10: 90})
        body = motors.add(13, 'XM430-W350-T', lower=BODY_LIMIT_MIN, upper=BODY_LIMIT_MAX)
        tail = motors.add(14, 'AX18-A', lower=TAIL_LIMIT_MIN, upper=TAIL_LIMIT_MAX, mirror=(14,))
        self.motors = motors
        self.behavior = None
        state = SimpleNamespace()
        state.step = 0
        state.substep = 0
        state.t = 0
        state.teleop = False
        state.teleop_key = None
        state.preload = True
        state.controller_display = ""
        state.behavior_display = ""
        state.weights = [1, 1, 1, 1, 1]
        state.gripper_angles = [0, 0, 0, 0]
        state.gripper_offsets = (45, 45, 45, 45)  # 0 = forward, 90 = outward
        # state.gripper_offsets = (90, 90, 90, 90)  # 0 = forward, 90 = outward
        state.kp, state.ki, state.kd, state.ko, state.kx, state.kr, state.km = KP, KI, KD, KO, KX, KR, KM
        self.state = state
        self.controller = controller
        self.fl = Leg(self, yaw[FL - 1], shoulder[FL - 1], knee[FL - 1], origin=(-WIDTH / 2, LENGTH / 2, HEIGHT/2),
                      mirror=True)
        self.fr = Leg(self, yaw[FR - 1], shoulder[FR - 1], knee[FR - 1], origin=(WIDTH / 2, LENGTH / 2, HEIGHT/2))
        self.rl = Leg(self, yaw[RL - 1], shoulder[RL - 1], knee[RL - 1], origin=(-WIDTH / 2, -LENGTH / 2, HEIGHT/2),
                      mirror=True)
        self.rr = Leg(self, yaw[RR - 1], shoulder[RR - 1], knee[RR - 1], origin=(WIDTH / 2, -LENGTH / 2, HEIGHT/2))
        self.legs = {FL: self.fl, FR: self.fr, RL: self.rl, RR: self.rr}
        self.body = BodyJoint(self, body[0])
        self.tail = TailJoint(self, tail[0], origin=(0, -LENGTH / 2, 0))

    def get_legs(self):
        return self.legs[1], self.legs[2], self.legs[3], self.legs[4]

    def set_behavior(self, behavior=None):
        """
        Change the robot's behavior, including any required termination or initialization
        """
        if self.behavior:
            self.behavior(self, term=True)  # Terminate old behavior
        self.behavior = behavior            # Change behaviors
        self.state.behavior_display = ""
        if self.behavior:
            self.behavior(self, init=True)  # Initialize new behavior

    def set_controller(self, controller):
        """
        Change the robot's controller, including any required initialization
        """
        self.controller = controller        # Change controllers
        self.state.controller_display = ""
        self.controller(self, init=True)    # Initialize new controller

    def transform(self, origin, point=(0, 0, 0), vector=False, inverse=False):
        """
        Transform a point from the origin frame to the centroidal frame
        :param origin: A point on the robot in mm relative to the centroid in the initial robot configuration
        :param point: A point relative to the origin point in the initial robot configuration
        :param vector: Point represents a vector quantity (do not translate from origin to centroid)
        :param inverse: Set to true to invert the transform of the point (origin is still in untransformed frame)
        :return: The transformed point relative to the centroid
        """
        point = np.array(point)
        if not inverse and not vector:
            point = point + origin
        a = self.transform_angle(origin, inverse)
        R = np.array(((1, 0, 0),
                      (0, cosd(a), -sind(a)),
                      (0, sind(a), cosd(a))))
        point = R @ point
        if inverse and not vector:
            point = point - origin
        return point

    def transform_angle(self, origin, inverse=False):
        """
        Determine pitch angle change from origin frame to centroidal frame
        :param origin: A point on the robot in mm relative to the centroid in the initial robot configuration
        :param inverse: Set to true to negate the transform angle (origin is still in untransformed frame)
        :return: The transform pitch angle in deg
        """
        flip = -1 if inverse else 1
        if origin[1] > 0:
            return self.body.get_angle() * flip
        else:
            return 0

    def get_pos(self, goal=False):
        """
        Determine position of centroid relative to the average of the feet positions
        :param goal: Return desired value instead of current value
        :return: Position of body in mm
        """
        return -np.sum([leg.get_pos(relative=False, goal=goal) for leg in self.legs.values()], axis=0)/4

    def move_to_position(self, dt, x=None, y=None, z=None, v=None):
        """
        Move body linearly towards a desired position relative to the feet
        :param dt: Time since last update in seconds
        :param x: Desired body x position in mm
        :param y: Desired body y position in mm
        :param z: Desired body z position in mm
        :param v: Moving speed in mm/s (defaults to current motor velocity limit)
        :return: True if body is at the desired position
        """
        current_pos = self.get_pos(goal=True)
        dx = 0 if x is None else x - current_pos[0]
        dy = 0 if y is None else y - current_pos[1]
        dz = 0 if z is None else z - current_pos[2]
        d = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        if v is None:
            v = np.min([np.min(np.abs(leg.get_vel(goal=True))) for leg in self.legs.values()])
        if d < v * dt:
            self.move_in_direction(dt, dx, dy, dz, d / dt)
            return True
        self.move_in_direction(dt, dx, dy, dz, v)
        return False

    def move_in_direction(self, dt, dx=0, dy=0, dz=0, v=None):
        """
        Move body linearly in given direction
        :param dt: Time since last update in seconds
        :param dx: Desired x component of direction
        :param dy: Desired y component of direction
        :param dz: Desired z component of direction
        :param v: Moving speed in mm/s (defaults to current motor velocity limit)
        :return: Movement speed in mm/s
        """
        if v is None:
            v = np.min([np.min(np.abs(leg.get_vel(goal=True))) for leg in self.legs.values()])
        for leg in self.legs.values():
            leg.move_in_direction(dt, -dx, -dy, -dz, v, relative=False)
        return v


class Leg:
    def __init__(self, parent, yaw_motor, shoulder_motor, knee_motor, origin=(0, 0, 0), mirror=False):
        """
        :param parent: Parent robot object
        :param yaw_motor: Yaw motor object
        :param shoulder_motor: Shoulder motor object
        :param knee_motor: Knee motor object
        :param origin: Shoulder position in mm relative to the centroid
        :param mirror: Set to true if leg is facing the negative x-axis
        """
        self.parent = parent
        self.yaw_motor, self.shoulder_motor, self.knee_motor = yaw_motor, shoulder_motor, knee_motor
        self.origin = origin
        self.mirror = mirror

    def get_pos(self, a1=None, a2=None, a3=None, relative=False, goal=False):
        """
        Get end effector position from given or current (default) joint angles

        :param a1: Yaw joint angle in deg
        :param a2: Shoulder joint angle in deg
        :param a3: Knee joint angle in deg
        :param relative: Evaluate in the shoulder frame instead of the centroidal frame
        :param goal: Return desired value instead of current value
        :return: Position of end effector in mm
        """
        if a1 is None:
            a1 = self.yaw_motor.goal_angle if goal else self.yaw_motor.angle
        if a2 is None:
            a2 = self.shoulder_motor.goal_angle if goal else self.shoulder_motor.angle
        if a3 is None:
            a3 = self.knee_motor.goal_angle if goal else self.knee_motor.angle
        x = cosd(a1) * (LINK1 - LINK3 * sind(a2 + a3) + LINK2A * cosd(a2) - LINK2B * sind(a2))
        y = sind(a1) * (LINK1 - LINK3 * sind(a2 + a3) + LINK2A * cosd(a2) - LINK2B * sind(a2))
        z = - LINK3 * cosd(a2 + a3) - LINK2B * cosd(a2) - LINK2A * sind(a2)
        if not relative:
            if self.mirror:
                x *= -1
            return self.parent.transform(self.origin, (x, y, z))
        return x, y, z

    def get_vel(self, w1=None, w2=None, w3=None, relative=False, goal=False):
        """
        Get end effector velocity from given or current (default) joint velocities

        :param w1: Yaw joint angular velocity in deg/s
        :param w2: Shoulder joint angular velocity in deg/s
        :param w3: Knee joint angular velocity in deg/s
        :param relative: Evaluate in the shoulder frame instead of the centroidal frame
        :param goal: Return desired value instead of current value
        :return: Velocity of end effector in mm/s
        """
        if w1 is None:
            w1 = self.yaw_motor.goal_velocity if goal else self.yaw_motor.velocity
        if w2 is None:
            w2 = self.shoulder_motor.goal_velocity if goal else self.shoulder_motor.velocity
        if w3 is None:
            w3 = self.knee_motor.goal_velocity if goal else self.knee_motor.velocity
        w1, w2, w3 = np.deg2rad(w1), np.deg2rad(w2), np.deg2rad(w3)
        a1, a2, a3 = self.get_angle()
        vx = -w2 * cosd(a1) * (LINK3 * cosd(a2 + a3) + LINK2B * cosd(a2) + LINK2A * sind(a2)) - w1 * sind(a1) * (
                LINK1 - LINK3 * sind(a2 + a3) + LINK2A * cosd(a2) - LINK2B * sind(a2)) - LINK3 * w3 * cosd(
            a2 + a3) * cosd(a1)
        vy = w1 * cosd(a1) * (LINK1 - LINK3 * sind(a2 + a3) + LINK2A * cosd(a2) - LINK2B * sind(a2)) - w2 * sind(a1) * (
                LINK3 * cosd(a2 + a3) + LINK2B * cosd(a2) + LINK2A * sind(a2)) - LINK3 * w3 * cosd(a2 + a3) * sind(a1)
        vz = w2 * (LINK3 * sind(a2 + a3) - LINK2A * cosd(a2) + LINK2B * sind(a2)) + LINK3 * w3 * sind(a2 + a3)
        if not relative:
            if self.mirror:
                vx *= -1
            return self.parent.transform(self.origin, (vx, vy, vz), vector=True)
        return vx, vy, vz

    def get_force(self, t1=None, t2=None, t3=None, relative=False, goal=False):
        """
        Get end effector contact force from given or current (default) joint torques

        :param t1: Yaw joint torque in Nmm
        :param t2: Shoulder joint torque in Nmm
        :param t3: Knee joint torque in Nmm
        :param relative: Evaluate in the shoulder frame instead of the centroidal frame
        :param goal: Return desired value instead of current value
        :return: Contact force applied by end effector in N
        """
        if t1 is None:
            t1 = self.yaw_motor.goal_torque if goal else self.yaw_motor.torque
        if t2 is None:
            t2 = self.shoulder_motor.goal_torque if goal else self.shoulder_motor.torque
        if t3 is None:
            t3 = self.knee_motor.goal_torque if goal else self.knee_motor.torque
        a1, a2, a3 = self.get_angle()
        fx = (t3 * cosd(a1) * (LINK3 * sind(a2 + a3) - LINK2A * cosd(a2) + LINK2B * sind(a2))) / (
                LINK3 * (LINK2A * cosd(a3) + LINK2B * sind(a3))) - (t2 * sind(a2 + a3) * cosd(a1)) / (
                LINK2A * cosd(a3) + LINK2B * sind(a3)) - (t1 * sind(a1)) / (
                LINK1 - LINK3 * sind(a2 + a3) + LINK2A * cosd(a2) - LINK2B * sind(a2))
        fy = (t1 * cosd(a1)) / (LINK1 - LINK3 * sind(a2 + a3) + LINK2A * cosd(a2) - LINK2B * sind(a2)) - (
                t2 * sind(a2 + a3) * sind(a1)) / (LINK2A * cosd(a3) + LINK2B * sind(a3)) + (
                t3 * sind(a1) * (LINK3 * sind(a2 + a3) - LINK2A * cosd(a2) + LINK2B * sind(a2))) / (
                LINK3 * (LINK2A * cosd(a3) + LINK2B * sind(a3)))
        fz = (LINK3 * t3 * cosd(a2 + a3) - LINK3 * t2 * cosd(a2 + a3) + LINK2B * t3 * cosd(a2) + LINK2A * t3 * sind(
            a2)) / (LINK3 * (LINK2A * cosd(a3) + LINK2B * sind(a3)))
        fy *= -1
        if not relative:
            if self.mirror:
                fx *= -1
            return self.parent.transform(self.origin, (fx, fy, fz), vector=True)
        return fx, fy, fz

    def get_angle(self, x=None, y=None, z=None, relative=False, goal=False):
        """
        Get joint angles from given or current (default) end effector position

        :param x: End effector x position in mm
        :param y: End effector y position in mm
        :param z: End effector z position in mm
        :param relative: Evaluate in the shoulder frame instead of the centroidal frame
        :param goal: Return desired value instead of current value
        :return: Joint angles in deg
        """
        if x is None and y is None and z is None:
            if goal:
                return self.yaw_motor.goal_angle, self.shoulder_motor.goal_angle, self.knee_motor.goal_angle
            else:
                return self.yaw_motor.angle, self.shoulder_motor.angle, self.knee_motor.angle
        if x is None or y is None or z is None:
            pos = self.get_pos(goal=goal)
            if x is None:
                x = pos[0]
            if y is None:
                y = pos[1]
            if z is None:
                z = pos[2]
        if not relative:
            if self.mirror:
                x *= -1
            x, y, z = self.parent.transform(self.origin, (x, y, z), inverse=True)
        # TODO: inverse kinematics
        return self.yaw_motor.angle, self.shoulder_motor.angle, self.knee_motor.angle

    def get_angvel(self, vx=None, vy=None, vz=None, relative=False, goal=False):
        """
        Get joint angular velocities from given or current (default) end effector velocity

        :param vx: End effector x velocity in mm/s
        :param vy: End effector y velocity in mm/s
        :param vz: End effector z velocity in mm/s
        :param relative: Evaluate in the shoulder frame instead of the centroidal frame
        :param goal: Return desired value instead of current value
        :return: Joint angular velocities in deg/s
        """
        if vx is None and vy is None and vz is None:
            if goal:
                return self.yaw_motor.goal_velocity, self.shoulder_motor.goal_velocity, self.knee_motor.goal_velocity
            else:
                return self.yaw_motor.velocity, self.shoulder_motor.velocity, self.knee_motor.velocity
        if vx is None or vy is None or vz is None:
            vel = self.get_vel(goal=goal)
            if vx is None:
                vx = vel[0]
            if vy is None:
                vy = vel[1]
            if vz is None:
                vz = vel[2]
        if not relative:
            if self.mirror:
                vx *= -1
            vx, vy, vz = self.parent.transform(self.origin, (vx, vy, vz), vector=True, inverse=True)
        a1, a2, a3 = self.get_angle()
        w1 = ((vy * cosd(a1) - vx * sind(a1)) / (
            LINK1 - LINK3 * sind(a2 + a3) + LINK2A * cosd(a2) - LINK2B * sind(a2)))
        w2 = (-(vz * cosd(a2) * cosd(a3) - vz * sind(a2) * sind(a3) + vx * cosd(a1) * cosd(a2) * sind(a3) + vx * cosd(
            a1) * cosd(a3) * sind(a2) + vy * cosd(a2) * sind(a1) * sind(a3) + vy * cosd(a3) * sind(a1) * sind(a2)) /
            (LINK2A * cosd(a3) + LINK2B * sind(a3)))
        w3 = ((LINK3 * vz * cosd(a2 + a3) + LINK2B * vz * cosd(a2) + LINK2A * vz * sind(a2) + LINK3 * vx * sind(
            a2 + a3) * cosd(a1) + LINK3 * vy * sind(a2 + a3) * sind(a1) - LINK2A * vx * cosd(a1) * cosd(
            a2) + LINK2B * vx * cosd(a1) * sind(a2) - LINK2A * vy * cosd(a2) * sind(a1) + LINK2B * vy * sind(a1) *
            sind(a2)) / (LINK3 * (LINK2A * cosd(a3) + LINK2B * sind(a3))))
        return np.rad2deg(w1), np.rad2deg(w2), np.rad2deg(w3)

    def get_torque(self, fx=None, fy=None, fz=None, relative=False, goal=False):
        """
        Get joint torques from given or current (default) end effector contact force

        :param fx: End effector x force in N
        :param fy: End effector y force in N
        :param fz: End effector z force in N
        :param relative: Evaluate in the shoulder frame instead of the centroidal frame
        :param goal: Return desired value instead of current value
        :return: Joint torques in Nmm
        """
        if fx is None and fy is None and fz is None:
            if goal:
                return self.yaw_motor.goal_torque, self.shoulder_motor.goal_torque, self.knee_motor.goal_torque
            else:
                return self.yaw_motor.torque, self.shoulder_motor.torque, self.knee_motor.torque
        if fx is None or fy is None or fz is None:
            force = self.get_force(goal=goal)
            if fx is None:
                fx = force[0]
            if fy is None:
                fy = force[1]
            if fz is None:
                fz = force[2]
        fy *= -1
        if not relative:
            if self.mirror:
                fx *= -1
            fx, fy, fz = self.parent.transform(self.origin, (fx, fy, fz), vector=True, inverse=True)
        a1, a2, a3 = self.get_angle()
        t1 = (fy * cosd(a1) - fx * sind(a1)) * (LINK1 - LINK3 * sind(a2 + a3) + LINK2A * cosd(a2) - LINK2B * sind(a2))
        t2 = fz * (LINK3 * sind(a2 + a3) - LINK2A * cosd(a2) + LINK2B * sind(a2)) - fx * cosd(a1) * (
            LINK3 * cosd(a2 + a3) + LINK2B * cosd(a2) + LINK2A * sind(a2)) - fy * sind(a1) * (
            LINK3 * cosd(a2 + a3) + LINK2B * cosd(a2) + LINK2A * sind(a2))
        t3 = LINK3 * fz * sind(a2 + a3) - LINK3 * fx * cosd(a2 + a3) * cosd(a1) - LINK3 * fy * cosd(a2 + a3) * sind(a1)
        return t1, t2, t3

    def set_pos(self, x=None, y=None, z=None, relative=False, direct=False):
        """
        Set end effector position

        :param x: End effector x position in mm
        :param y: End effector y position in mm
        :param z: End effector z position in mm
        :param direct: Modify the motor value directly instead of changing the setpoint
        :param relative: Evaluate in the shoulder frame instead of the centroidal frame
        """
        self.set_angle(*self.get_angle(x, y, z, relative, goal=True), direct=direct)

    def set_vel(self, vx=None, vy=None, vz=None, relative=False, direct=False):
        """
        Set end effector velocity

        :param vx: End effector x velocity in mm/s
        :param vy: End effector y velocity in mm/s
        :param vz: End effector z velocity in mm/s
        :param direct: Modify the motor value directly instead of changing the setpoint
        :param relative: Evaluate in the shoulder frame instead of the centroidal frame
        """
        self.set_angvel(*self.get_angvel(vx, vy, vz, relative, goal=True), direct=direct)

    def set_force(self, fx=None, fy=None, fz=None, relative=False, direct=False):
        """
        Set end effector contact force

        :param fx: End effector x force in N
        :param fy: End effector y force in N
        :param fz: End effector z force in N
        :param direct: Modify the motor value directly instead of changing the setpoint
        :param relative: Evaluate in the shoulder frame instead of the centroidal frame
        """
        self.set_torque(*self.get_torque(fx, fy, fz, relative, goal=True), direct=direct)

    def set_angle(self, a1=None, a2=None, a3=None, direct=False):
        """
        Set joint angles

        :param a1: Yaw joint angle in deg
        :param a2: Shoulder joint angle in deg
        :param a3: Knee joint angle in deg
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        if a1 is not None:
            if direct:
                self.yaw_motor.set_angle = a1
            else:
                self.yaw_motor.goal_angle = a1
        if a2 is not None:
            if direct:
                self.shoulder_motor.set_angle = a2
            else:
                self.shoulder_motor.goal_angle = a2
        if a3 is not None:
            if direct:
                self.knee_motor.set_angle = a3
            else:
                self.knee_motor.goal_angle = a3

    def set_angvel(self, w1=None, w2=None, w3=None, direct=False):
        """
        Set joint angular velocities

        :param w1: Yaw joint angular velocity in deg/s
        :param w2: Shoulder joint angular velocity in deg/s
        :param w3: Knee joint angular velocity in deg/s
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        if w1 is not None:
            if direct:
                self.yaw_motor.set_velocity = w1
            else:
                self.yaw_motor.goal_velocity = w1
        if w2 is not None:
            if direct:
                self.shoulder_motor.set_velocity = w2
            else:
                self.shoulder_motor.goal_velocity = w2
        if w3 is not None:
            if direct:
                self.knee_motor.set_velocity = w3
            else:
                self.knee_motor.goal_velocity = w3

    def set_torque(self, t1=None, t2=None, t3=None, direct=False):
        """
        Set joint torques

        :param t1: Yaw joint torque in Nmm
        :param t2: Shoulder joint torque in Nmm
        :param t3: Knee joint torque in Nmm
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        scale = 1
        if t1:
            scale = max(scale, abs(t1)/self.yaw_motor.stall)
        if t2:
            scale = max(scale, abs(t2)/self.shoulder_motor.stall)
        if t3:
            scale = max(scale, abs(t3)/self.knee_motor.stall)

        if t1 is not None:
            if direct:
                self.yaw_motor.set_torque = t1/scale
            else:
                self.yaw_motor.goal_torque = t1/scale
        if t2 is not None:
            if direct:
                self.shoulder_motor.set_torque = t2/scale
            else:
                self.shoulder_motor.goal_torque = t2/scale
        if t3 is not None:
            if direct:
                self.knee_motor.set_torque = t3/scale
            else:
                self.knee_motor.goal_torque = t3/scale

    def move_to_position(self, dt, x=None, y=None, z=None, v=None, relative=False):
        """
        Move foot linearly towards a desired position
        :param dt: Time since last update in seconds
        :param x: Desired foot x position in mm
        :param y: Desired foot y position in mm
        :param z: Desired foot z position in mm
        :param v: Moving speed in mm/s (defaults to current motor velocity limit)
        :param relative: Desired position is relative to shoulder instead of centroid
        :return: True if foot is at the desired position
        """
        current_pos = self.get_pos(relative=relative, goal=True)
        dx = 0 if x is None else x - current_pos[0]
        dy = 0 if y is None else y - current_pos[1]
        dz = 0 if z is None else z - current_pos[2]
        d = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        v_max = self.get_max_speed(dx/d, dy/d, dz/d, relative)
        if v is None or v > v_max:
            v = v_max
        if d < v * dt:
            self.move_in_direction(dt, dx, dy, dz, d / dt, relative=relative)
            return True
        self.move_in_direction(dt, dx, dy, dz, v, relative=relative)
        return False

    def move_in_direction(self, dt, dx=0, dy=0, dz=0, v=None, relative=False, direct=False):
        """
        Move foot linearly in given direction
        :param dt: Time since last update in seconds
        :param dx: Desired x component of direction
        :param dy: Desired y component of direction
        :param dz: Desired z component of direction
        :param v: Moving speed in mm/s (defaults to current motor velocity limit)
        :param relative: Desired direction is relative to shoulder instead of centroid
        :param direct: Modify the motor value but do not change the setpoint
        :return: Movement speed in mm/s
        """
        d = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        if d <= 0:
            return
        v_max = self.get_max_speed(dx/d, dy/d, dz/d, relative)
        if v is None or v > v_max:
            v = v_max
        angvel = np.array(self.get_angvel(v*dx/d, v*dy/d, v*dz/d, relative=relative))
        angle = self.yaw_motor.set_angle, self.shoulder_motor.set_angle, self.knee_motor.set_angle
        self.set_angle(*(angle + angvel * dt), direct=True)
        if not direct:
            angle = self.get_angle(goal=True)
            self.set_angle(*(angle + angvel * dt), direct=False)
        return v

    def get_max_speed(self, dx, dy, dz, relative):
        """
        Determine the maximum speed the foot can move in a given direction given the current angular velocity limits
        :return: Maximum speed in millimeters per second
        """
        w = self.get_angvel(dx, dy, dz, relative=relative)
        w_max = np.array(self.get_angvel(relative=True, goal=True))
        if not w_max[0]:
            w_max[0] = self.yaw_motor.speed
        if not w_max[1]:
            w_max[1] = self.shoulder_motor.speed
        if not w_max[2]:
            w_max[2] = self.knee_motor.speed
        return 1 / np.amax(np.abs(w / w_max))  # Fastest linear speed that obeys motor angular velocity limits


class TailJoint:
    def __init__(self, parent, motor, origin=(0, 0, 0)):
        """
        :param parent: Parent robot object
        :param motor: Tail motor object
        :param origin: Tail motor position in mm relative to the centroid
        """
        self.parent = parent
        self.motor = motor
        self.origin = origin

    def get_pos(self, a=None, relative=False, goal=False):
        """
        Get tail tip position from given or current (default) joint angle

        :param a: Tail joint angle in deg
        :param relative: Evaluate in the tail motor frame instead of the centroidal frame
        :param goal: Return desired value instead of current value
        :return: Position of tail tip in mm
        """
        if a is None:
            a = self.get_angle(goal=goal)
        pos = np.array((0, -TAIL * cosd(a), TAIL * sind(a)))
        if not relative:
            pos += self.origin
        return pos

    def get_vel(self, w=None, goal=False):
        """
        Get tail tip velocity from given or current (default) joint velocity

        :param w: Tail joint angular velocity in deg/s
        :param goal: Return desired value instead of current value
        :return: Velocity of tail tip in mm/s
        """
        if w is None:
            w = self.get_angvel(goal=goal)
        a = self.get_angle()
        vel = np.array((0, TAIL * sind(a) * np.deg2rad(w), TAIL * cosd(a) * np.deg2rad(w)))
        return vel

    def get_force(self, t=None, goal=False):
        """
        Get tail tip contact force from given or current (default) joint torques

        :param t: Tail joint torque in Nmm
        :param goal: Return desired value instead of current value
        :return: Contact force applied by tail tip in N
        """
        if t is None:
            t = self.get_torque(goal=goal)
        a = self.get_angle()
        force = np.array((0, t * sind(a) / TAIL, t * cosd(a) / TAIL))
        return force

    def get_angle(self, z=None, goal=False):
        """
        Get tail joint angle from given or current (default) tail tip z position

        :param z: Tail tip z position in mm
        :param goal: Return desired value instead of current value
        :return: Tail joint angle in deg
        """
        if z is None:
            return self.motor.goal_angle if goal else self.motor.angle
        return np.rad2deg(np.arcsin(z / TAIL))

    def get_angvel(self, vz=None, goal=False):
        """
        Get tail angular velocity from given or current (default) tail tip z velocity

        :param vz: Tail tip z velocity in mm/s
        :param goal: Return desired value instead of current value
        :return: Tail joint angular velocity in deg/s
        """
        if vz is None:
            return self.motor.goal_velocity if goal else self.motor.velocity
        a = self.get_angle()
        return np.rad2deg(vz / TAIL / cosd(a))

    def get_torque(self, fx=None, fy=None, fz=None, goal=False):
        """
        Get tail joint torque from given or current (default) tail tip contact force

        :param fx: Tail tip x force in N
        :param fy: Tail tip y force in N
        :param fz: Tail tip z force in N
        :param goal: Return desired value instead of current value
        :return: Tail joint torque in Nmm
        """
        if fz is None:
            return self.motor.goal_torque if goal else self.motor.torque
        if fy is None:
            fy = 0
        a = self.get_angle()
        return fz * TAIL * cosd(a) + fy * TAIL * sind(a)

    def set_pos(self, z, direct=False):
        """
        Set tail tip z position

        :param z: Tail tip z position in mm
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        self.set_angle(self.get_angle(z, goal=True), direct=direct)

    def set_vel(self, vz, direct=False):
        """
        Set tail tip z velocity

        :param vz: Tail tip z velocity in mm/s
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        self.set_angvel(self.get_angvel(vz, goal=True), direct=direct)

    def set_force(self, fz, direct=False):
        """
        Set tail tip contact force

        :param fz: Tail tip z force in N
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        self.set_torque(self.get_torque(fz=fz, goal=True), direct=direct)

    def set_angle(self, a, direct=False):
        """
        Set tail joint angle

        :param a: tail joint angle in deg
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        if direct:
            self.motor.set_angle = a
        else:
            self.motor.goal_angle = a

    def set_angvel(self, w, direct=False):
        """
        Set tail joint angular velocity

        :param w: tail joint angular velocity in deg/s
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        if direct:
            self.motor.set_velocity = w
        else:
            self.motor.goal_velocity = w

    def set_torque(self, t, direct=False):
        """
        Set tail joint torque

        :param t: tail joint torque in Nmm
        :param direct: Modify the motor value directly instead of changing the setpoint
        """

        t_max = 700
        if t > t_max:
            t = t_max
        if t < -t_max:
            t = -t_max
        if direct:
            self.motor.set_torque = t
        else:
            self.motor.goal_torque = t


class BodyJoint:
    def __init__(self, parent, motor, origin=(0, 0, 0)):
        """
        :param parent: Parent robot object
        :param motor: Body motor object
        :param origin: Body motor position in mm relative to the centroid
        """
        self.parent = parent
        self.motor = motor
        self.origin = origin

    def get_angle(self, goal=False):
        """
        Get body joint angle

        :param goal: Return desired value instead of current value
        :return: Body joint angle in deg
        """
        return self.motor.goal_angle if goal else self.motor.angle

    def get_angvel(self, goal=False):
        """
        Get body joint angular velocity

        :param goal: Return desired value instead of current value
        :return: Body joint angular velocity in deg/s
        """
        return self.motor.goal_velocity if goal else self.motor.velocity

    def get_torque(self, goal=False):
        """
        Get body joint torque

        :param goal: Return desired value instead of current value
        :return: Body joint torque in Nmm
        """
        return self.motor.goal_torque if goal else self.motor.torque

    def set_angle(self, a, direct=False):
        """
        Set body joint angle

        :param a: body joint angle in deg
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        if direct:
            self.motor.set_angle = a
        else:
            self.motor.goal_angle = a

    def set_angvel(self, w, direct=False):
        """
        Set body joint angular velocity

        :param w: body joint angular velocity in deg/s
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        if direct:
            self.motor.set_velocity = w
        else:
            self.motor.goal_velocity = w

    def set_torque(self, t, direct=False):
        """
        Set body joint torque

        :param t: body joint torque in Nmm
        :param direct: Modify the motor value directly instead of changing the setpoint
        """
        if direct:
            self.motor.set_torque = t
        else:
            self.motor.goal_torque = t


if __name__ == "__main__":
    from motors import Motors

    dxl = Motors()
    robot = Robot(dxl)

    def test(val, goal):
        if np.all(np.abs(np.array(val) - goal) < 1e-6):
            print("PASS")
        else:
            print("FAIL: " + str(val) + " != " + str(goal))

    # Controller tests
    print("\nController Tests:")
    test(get_position_vector(robot).shape, (13,))
    test(get_grasp_map(robot).shape, (6, 13))
    robot.state.gripper_angles = (15, 15, 15, 15)
    print(hang_qp(robot, np.array((0, 30, 0, -2000, 0, 0))))

    # Tail tests
    print("\nTail Kinematics Tests:")
    robot.tail.motor.angle = 7
    test(robot.tail.get_angle(robot.tail.get_pos(42)[2]), 42)
    test(robot.tail.get_pos(robot.tail.get_angle(42))[2], 42)
    test(robot.tail.get_angvel(robot.tail.get_vel(42)[2]), 42)
    test(robot.tail.get_vel(robot.tail.get_angvel(42))[2], 42)
    test(robot.tail.get_torque(*robot.tail.get_force(42)), 42)
    test(robot.tail.get_torque(*robot.tail.get_force(robot.tail.get_torque(fz=42))), robot.tail.get_torque(fz=42))

    # Leg tests
    print("\nLeg Kinematics Tests:")
    robot.fl.yaw_motor.angle = 7
    robot.fl.shoulder_motor.angle = 14
    robot.fl.knee_motor.angle = 21
    robot.body.motor.angle = 0
    test(robot.fl.get_angle(*robot.fl.get_pos(42, 43, 44)), (42, 43, 44))
    test(robot.fl.get_pos(*robot.fl.get_angle(42, 43, 44)), (42, 43, 44))
    test(robot.fl.get_angvel(*robot.fl.get_vel(42, 43, 44)), (42, 43, 44))
    test(robot.fl.get_vel(*robot.fl.get_angvel(42, 43, 44)), (42, 43, 44))
    test(robot.fl.get_torque(*robot.fl.get_force(42, 43, 44)), (42, 43, 44))
    test(robot.fl.get_force(*robot.fl.get_torque(42, 43, 44)), (42, 43, 44))
    test(robot.transform(robot.fl.origin, robot.transform(robot.fl.origin, (42, 43, 44), inverse=True)), (42, 43, 44))
    test(robot.transform(robot.fl.origin, robot.transform(robot.fl.origin, (42, 43, 44)), inverse=True), (42, 43, 44))
