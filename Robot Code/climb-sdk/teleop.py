"""
Parse user input to control the robot
"""

import matplotlib.pyplot as plt
import sys

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
        self.stdout = sys.stdout
        self.i = 0
        self.log = np.zeros((16, 0))
        sys.stdout = self.buffer = buffer

    def log_force(self, robot, t):
        leg_forces = np.array(list(leg.get_force(relative=True) for leg in robot.get_legs())).flatten()
        tail_force = robot.tail.get_force()
        data = np.vstack((t, *leg_forces, *tail_force))
        self.log = np.append(self.log, data, axis=1)

    def plot_force(self, foot=(1, 2, 3, 4, 5), axis=(0, 1, 2), **kwargs):
        w = 10
        log = np.zeros((self.log.shape[0], self.log.shape[1] - w + 1))
        for i in range(0, self.log.shape[0]):
            log[i, :] = np.convolve(self.log[i, :], np.ones(w), 'valid') / w
        log[0, :] -= log[0, 0]

        foot_labels = ("", "FL", "FR", "RL", "RR", "Tail")
        axis_labels = ("Lateral", "Tangential", "Normal")
        if hasattr(foot, "__iter__"):
            for f in foot:
                if hasattr(axis, "__iter__"):
                    for a in axis:
                        i = f * 3 + a - 2
                        label = f'{foot_labels[f]} {axis_labels[a]}'
                        plt.plot(log[0, :], log[i, :], label=label, **kwargs)
                else:
                    i = f * 3 + axis - 2
                    label = f'{foot_labels[f]}'
                    plt.plot(log[0, :], log[i, :], label=label, **kwargs)
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
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.show()

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
            self.stdout.write(buffer[self.i:])
            self.i = len(buffer)

    def execute(self, command, robot):
        """
        Execute the given command
        :param command: Command string
        :param robot: Robot to command
        """
        c = command[0]
        if robot.state.teleop:
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
            sys.stdout = self.stdout
            self.quit = True
        elif c == " ":
            robot.set_behavior(None)
            print(t + "Idle")
        elif c == "u":
            robot.set_behavior(stand)
            print(t + "Stand")
        elif c == "s":
            robot.set_behavior(stick)
            print(t + "Stick")
        elif c == "w":
            robot.set_behavior(walk)
            print(t + "Walk")
        elif c == "c":
            robot.set_behavior(climb2)
            print(t + "Climb")
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
        elif c == "r":
            print(t + "Retry step")
            robot.state.step -= 1
            robot.set_behavior(climb)
        elif c == ";":
            print(t + "Plotting force")
            self.plot_force(axis=0)
            self.plot_force(axis=1)
            self.plot_force(axis=2)
            self.plot_force(foot=3)
        elif c == " ":
            if robot.state.teleop:
                print(t + "Teleop completed")
                robot.state.teleop = False
        elif c == "o":
            if robot.controller == control_off:
                robot.set_controller(centralized_admittance)
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
    f = [leg_function(leg, **kwargs) for leg in robot.get_legs()]
    f.append(tail_function(robot.tail))
    print(", ".join([f'{fi:4.1f}' for fi in np.sum(np.array(f), 0)]))

    display = " | ".join([", ".join([f'{f:4.1f}' for f in leg_function(leg, **kwargs)]) for leg in robot.get_legs()])
    if tail_function:
        display += " | " + ", ".join([f'{f:4.1f}' for f in tail_function(robot.tail, **kwargs)])
    return display
