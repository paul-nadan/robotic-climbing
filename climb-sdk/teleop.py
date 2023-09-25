"""
Parse user input to control the robot
"""
import datetime

import matplotlib.pyplot as plt
import sys
import time

from behaviors import *


class Terminal:

    def __init__(self, terminal, buffer):
        """
        Initialize terminal as user interface
        :param terminal: Curses window object
        """
        self.terminal = terminal
        self.terminal.nodelay(True)
        self.command = ""
        self.command_text = ""
        self.status = ["Loading", "", ""]
        self.t = 0
        self.quit = False
        self.i = 0
        self.log = np.zeros((31, 0))
        sys.stdout = self.buffer = buffer

    def log_force(self, robot, t):
        """
        Record the current measured and setpoint forces for future export and plotting
        """
        leg_forces = np.array(list(leg.get_force(relative=True) for leg in robot.get_legs())).flatten()
        tail_force = robot.tail.get_force()
        if len(robot.state.f_goal):
            goal_force = np.reshape(robot.state.f_goal, -1, order='F')
            goal_force[0] *= -1
            goal_force[6] *= -1
        else:
            goal_force = np.zeros(15)
        data = np.vstack((t, *leg_forces, *tail_force, *goal_force))
        self.log = np.append(self.log, data, axis=1)

    def plot_force(self, foot=(1, 2, 3, 4, 5), axis=(0, 1, 2), cost=0, **kwargs):
        """
        Plot the force on each foot
        Log data format: t x1 y1 z1 ... x4 y4 z4 xt yt zt xg1 yg1 zg1 ...
        """
        w = 10
        log = np.zeros((self.log.shape[0], self.log.shape[1] - w + 1))
        for i in range(0, self.log.shape[0]):
            log[i, :] = np.convolve(self.log[i, :], np.ones(w), 'valid') / w
        log[0, :] -= log[0, 0]

        foot_labels = ("", "FL", "FR", "RL", "RR", "Tail")
        axis_labels = ("Lateral", "Tangential", "Normal")
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        if cost:
            if hasattr(foot, "__iter__"):
                for f in (1, 2, 3, 4):
                    x = log[f * 3 - 2, :]
                    y = log[f * 3 - 1, :]
                    z = log[f * 3 - 0, :]
                    xg = log[f * 3 - 2 + 15, :]
                    yg = log[f * 3 - 1 + 15, :]
                    zg = log[f * 3 - 0 + 15, :]
                    label = f'{foot_labels[f]}'
                    mag = np.sqrt(x ** 2 + y ** 2 + np.maximum(0, -z) ** 2)
                    ang = np.rad2deg(np.arctan2(np.maximum(0, -z), np.sqrt(x ** 2 + y ** 2)))
                    mag_g = np.sqrt(xg ** 2 + yg ** 2 + np.maximum(0, -zg) ** 2)
                    ang_g = np.rad2deg(np.arctan2(np.maximum(0, -zg), np.sqrt(xg ** 2 + yg ** 2)))
                    if cost == 1:
                        p = plt.plot(log[0, :], mag, label=label, **kwargs)
                        c = p[0].get_color()
                        plt.plot(log[0, :], mag_g, linestyle='dashed', color=c, **kwargs)
                        plt.title('Adhesion Magnitude')
                    elif cost == 2:
                        p = plt.plot(log[0, :], ang, label=label, **kwargs)
                        c = p[0].get_color()
                        plt.plot(log[0, :], ang_g, linestyle='dashed', color=c, **kwargs)
                        plt.title('Adhesion Angle')
                        plt.ylabel('Angle (deg)')
                plt.legend()
        elif hasattr(foot, "__iter__"):
            for f in foot:
                if hasattr(axis, "__iter__"):
                    for a in axis:
                        i = f * 3 + a - 2
                        label = f'{foot_labels[f]} {axis_labels[a]}'
                        p = plt.plot(log[0, :], log[i, :], label=label, **kwargs)
                        c = p[0].get_color()
                        plt.plot(log[0, :], log[i + 15, :], linestyle='dashed', color=c, **kwargs)
                else:
                    i = f * 3 + axis - 2
                    label = f'{foot_labels[f]}'
                    p = plt.plot(log[0, :], log[i, :], label=label, **kwargs)
                    c = p[0].get_color()
                    plt.plot(log[0, :], log[i + 15, :], linestyle='dashed', color=c, **kwargs)
                    plt.title(f'{axis_labels[axis]} Force')
            plt.legend()
        else:
            if hasattr(axis, "__iter__"):
                for a in axis:
                    i = foot * 3 + a - 2
                    label = f'{axis_labels[a]}'
                    plt.plot(log[0, :], log[i, :], label=label, **kwargs)
                plt.title(f'{foot_labels[foot]} Force')
                plt.legend()
            else:
                i = foot * 3 + axis - 2
                plt.plot(log[0, :], log[i, :], **kwargs)
                plt.title(f'{foot_labels[foot]} {axis_labels[axis]} Force')
        plt.show()

    def save_log(self):
        """
        Export all force data to a time-stamped CSV
        """
        date = "{date:%Y-%m-%d-%H-%M-%S}".format(date=datetime.datetime.now())
        name = f'log/Force_Data_{date}.csv'
        try:
            np.savetxt(name, self.log.T, delimiter=',', header="t, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4, "
                                                               + "xt, yt, zt, gx1, gy1, gz1, gx2, gy2, gz2, gx3, "
                                                               + "gy3, gz3, gx4, gy4, gz4, gxt, gyt, gzt", comments="")
        except FileNotFoundError:
            print(f"Could not save file as {name}")

    def teleop(self, robot, dt):
        """
        Check for user input and command robot appropriately
        :param robot: Robot to command
        :param dt: Time since last teleop update
        """
        c = self.terminal.getch()
        if c != -1:
            c = chr(c)
            self.t = 0
            if self.command or c in "mvtk":  # continue longer command
                if c == "\n":  # submit command
                    self.execute(self.command, robot)
                    self.command = ""
                    self.command_text = ""
                elif c == "\b":  # backspace
                    self.command = self.command[:-1]
                else:  # continue command
                    self.command += c
            elif c != "\n":  # start new command
                self.execute(c, robot)
                self.command = ""
                self.command_text = c
        self.t += dt
        if self.command:
            self.command_text = self.command
        elif self.t > 0.5:
            self.command_text = ""

    def display(self):
        """
        Update the user interface display
        """
        buffer = self.buffer.getvalue()
        n = sum([1 if s else 0 for s in self.status])
        self.terminal.erase()
        y, x = self.terminal.getmaxyx()
        lines = buffer[-10000:].split("\n")[-y + n:-1]
        for i in range(y - 1 - n - len(lines)):
            self.terminal.addstr("\n")
        for line in lines:
            self.terminal.addstr(line[:x - 1] + "\n")
        for i in range(len(self.status)):
            if self.status[i]:
                self.terminal.addstr((self.status[i].replace("\n", " \\ ") + "\n")[:x - 1])
        self.terminal.addstr(("> " + self.command_text)[:x - 1])
        if self.i < len(buffer):
            sys.__stdout__.write(buffer[self.i:])
            self.i = len(buffer)

    def execute(self, command, robot):
        """
        Execute the given command
        :param command: Command string
        :param robot: Robot to command
        """
        c = command[0]
        if robot.state.teleop or (display_name(robot.behavior) == "Climb" and c in "wasdqeQEAD"):
            robot.state.teleop_key = c
            return
        t = f'{time.perf_counter() - self.log[0, 0]:.3f}: '
        try:
            if c in "mvt":
                s = command[1:].replace("=", " ").strip(" \t=").split(" ")
                id = int(s[0].strip(" \t="))
                val = float(s[1].strip(" \t="))
                if id not in robot.motors.motors_by_id:
                    return
                if c == "m":
                    print(t + f"Setting motor {id} to angle {val} degrees")
                    robot.motors.motors_by_id[id].goal_angle = val
                elif c == "v":
                    print(t + f"Setting motor {id} to speed {val} degrees per second")
                    robot.motors.motors_by_id[id].goal_velocity = val
                elif c == "t":
                    print(t + f"Setting motor {id} to torque {val} Newton millimeters")
                    robot.motors.motors_by_id[id].goal_torque = val
            elif c == "k":
                s = command[1:2].lower()
                val = float(command[2:].strip(" \t="))
                if s == "p":
                    robot.state.kp = val
                elif s == "i":
                    robot.state.ki = val
                elif s == "d":
                    robot.state.kd = val
                elif s == "o":
                    robot.state.ko = val
                elif s == "x":
                    robot.state.kx = val
                elif s == "f":
                    robot.state.kf = val
                elif s == "r":
                    robot.state.kr = val
                elif s == "m":
                    robot.state.km = val
                else:
                    print(t + f"Invalid command: {command}")
                    return
                print(t + f"Set k{s.upper()} to {val}")
        except (ValueError, IndexError):
            print(t + f"Invalid command: {command}")
            pass
        if c == "l":  # quit
            sys.stdout = sys.__stdout__
            self.quit = True
        elif c == " ":
            robot.set_behavior(None)
            print(t + "Idle")
        elif c == "u":
            robot.set_behavior(stand)
            print(t + "Stand")
        elif c == "s":
            robot.set_behavior(stick)
            robot.set_controller(control_off)
            print(t + "Stick")
        elif c == "w":
            robot.set_behavior(walk)
            print(t + "Walk")
        elif c == "c":
            robot.set_behavior(climb)
            print(t + "Climb")
        elif c == "z":
            robot.set_behavior(test_grasp)
            print(t + "Grasp Test")
        elif c == "x":
            robot.set_behavior(recenter)
            print(t + "Recenter")
        elif c == "p":
            robot.state.preload = not robot.state.preload
            if robot.state.preload:
                print(t + "Preload on")
            else:
                print(t + "Preload off")
        elif c == "e":
            if not robot.motors.opened:
                print(t + "Reconnect")
                robot.motors.connect()
            print(t + "Enable")
            robot.motors.enable()
        elif c == "y":
            print("Reset log")
            self.log = np.zeros((self.log.shape[0], 0))
        # elif c in "1234":
        #     i = int(c)
        #     if robot.state.weights[i - 1] > 0:
        #         print(f"Unload {i}")
        #         robot.set_behavior(unload, leg=i, reload=False)
        #     else:
        #         print(f"Reload {i}")
        #         robot.set_behavior(unload, leg=i, reload=True)
        elif c in "1234":
            i = int(c)
            print(t + "Retry step " + c)
            steps = [1, 3, 0, 2]
            robot.state.step = steps[i - 1]
            robot.set_behavior(climb)
        elif c == "r":
            print(t + "Retry step")
            robot.state.step -= 1
            robot.set_behavior(climb)
        elif c == ";":
            print(t + "Plotting force")
            self.plot_force(axis=0)
            self.plot_force(axis=1)
            self.plot_force(axis=2)
            self.plot_force(foot=1)
            self.save_log()
        elif c == " ":
            if robot.state.teleop:
                print(t + "Teleop completed")
                robot.state.teleop = False
        elif c == "0":
            print(t + "Zero torque")
            for leg in robot.legs.values():
                leg.set_torque(0, 0, 0)
            robot.tail.set_torque(0)
        elif c == "o":
            if robot.controller == control_off:
                robot.set_controller(admittance)
                reset_torque(robot)
                print(t + "Control On")
            else:
                robot.set_controller(control_off)
                print(t + "Control Off")


def display_name(f):
    """
    Returns the display name of the given function
    """
    if not f:
        return "Idle"
    return " ".join([n[0].upper() + n[1:] for n in f.__name__.split("_")])


def display_variable(robot, leg_function, tail_function=None, **kwargs):
    """
    Evaluates the specified function and re-formats the value for easier display
    """
    f = [leg_function(leg, **kwargs) for leg in robot.get_legs()]
    f.append(tail_function(robot.tail))
    print(", ".join([f'{fi:4.1f}' for fi in np.sum(np.array(f), 0)]))

    display = " | ".join([", ".join([f'{f:4.1f}' for f in leg_function(leg, **kwargs)]) for leg in robot.get_legs()])
    if tail_function:
        display += " | " + ", ".join([f'{f:4.1f}' for f in tail_function(robot.tail, **kwargs)])
    return display
