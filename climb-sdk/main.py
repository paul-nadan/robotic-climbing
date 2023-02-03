#!/usr/bin/env python3
import sys

from motors import Motors
from robot import Robot
from behaviors import *
from teleop import Terminal, display_name
from serial import SerialException
import io
import time
import curses           # pip install windows-curses
import os


def main_loop(terminal, buffer):
    interface = Terminal(terminal, buffer)
    if os.name == 'nt':
        port = "COM6"           # Windows
    else:
        port = "/dev/ttyUSB0"   # Linux
    robot = Robot(Motors(port=port, baud=1000000))
    robot.motors.enable()
    robot.tail.set_torque(robot.tail.get_torque(goal=True), direct=True)  # apply torque limits
    robot.set_behavior(sprawl)
    robot.set_controller(control_off)

    t = time.perf_counter()     # current time in seconds
    t0 = t                      # start time for loop counter in seconds
    loops = 0                   # loop counter for timing code
    while not interface.quit:
        robot.motors.read_angle()   # 12 ms
        robot.motors.read_torque()  # 12 ms

        dt = time.perf_counter() - t
        t += dt
        if dt > 1:              # Timeout
            continue

        interface.teleop(robot, dt)     # Check for operator input
        interface.display()             # Update the display
        if robot.behavior:
            robot.behavior(robot, dt)   # 1 ms
        robot.controller(robot, dt)     # 2 ms (admittance) + 1.5 ms (QP)

        robot.motors.write_angle()      # 1 ms
        robot.motors.write_torque()     # 1 ms

        loops += 1
        interface.log_force(robot, t)

        # Update the status
        if t - t0 > 0.25:
            robot.motors.read_voltage()
            robot.motors.read_temp()
            temp = max(m.temperature for m in robot.motors.get())
            volt = min(m.voltage for m in robot.motors.get())
            interface.status[0] = f"--- {robot.motors.status} | " \
                                  f"{display_name(robot.controller)} | " \
                                  f"{display_name(robot.behavior)} | " \
                                  f"{(t - t0) / loops * 1000:.2f} ms | " \
                                  f"{temp}°C | " \
                                  f"{volt}V ---"
            interface.status[1] = str(robot.state.controller_display)
            interface.status[2] = str(robot.state.behavior_display)
            t0 = t
            loops = 0

            # Temperature failsafe
            if temp > 70:   # AX limit is 75, XM limit is 80
                robot.motors.disable()
                for motor in robot.motors.get():
                    if motor.temperature > 70:
                        print(f"TEMPERATURE UNSAFE: motor {motor.id} at {motor.temperature}°C")


if __name__ == "__main__":
    np.set_printoptions(precision=2, suppress=True)
    buf = io.StringIO()
    try:
        curses.wrapper(lambda terminal: main_loop(terminal, buf))
        os.system('cls' if os.name == 'nt' else 'clear')
        log = buf.getvalue()
        for s in log:
            print(s, end="")
    except SerialException:
        print("Disconnected")
    finally:
        sys.stdout = sys.__stdout__
