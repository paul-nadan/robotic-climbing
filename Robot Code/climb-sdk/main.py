#!/usr/bin/env python3

from motors import Motors
from robot import Robot, Leg, TailJoint
from behaviors import *
from teleop import Terminal, display_name, display_variable
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
    robot.set_behavior(stick)
    robot.set_controller(control_off)

    t = time.perf_counter()     # current time in seconds
    t0 = t                      # start time for loop counter in seconds
    loops = 0                   # loop counter for timing code
    while not interface.quit:
        robot.motors.read_angle()  # 12 ms
        robot.motors.read_torque()  # 12 ms

        dt = time.perf_counter() - t
        t += dt
        if dt > 1:  # Timeout
            continue

        interface.teleop(robot, dt)
        interface.display()
        if robot.behavior:
            robot.behavior(robot, dt)  # 0-1 ms
        robot.controller(robot, dt)  # 2 ms (admittance) + 1.5 ms (QP)

        robot.motors.write_angle()  # 1 ms
        robot.tail.set_torque(robot.tail.get_torque(goal=True), direct=True)
        robot.motors.write_torque()  # 1 ms

        loops += 1
        # print(str(t) + "\t" + "\t".join([f'{f:.4f}' for f in robot.fl.get_force(relative=True)]))
        interface.log_force(robot, t)
        # print(display_variable(robot, Leg.get_force, TailJoint.get_force))
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
            # print([[f'F {f:.1f}' for f in leg.get_force(relative=False)] for leg in robot.get_legs()])
            # print([[f'T {f:.1f}' for f in leg.get_force(relative=True)] for leg in robot.get_legs()])
            t0 = t
            loops = 0
            if temp > 70:   # AX limit is 75, XM limit is 80
                robot.motors.disable()
                for motor in robot.motors.get():
                    if motor.temperature > 70:
                        print(f"TEMPERATURE OVERRIDE: motor {motor.id} at {motor.temperature}°C")


if __name__ == "__main__":
    buf = io.StringIO()
    try:
        curses.wrapper(lambda terminal: main_loop(terminal, buf))
        os.system('cls' if os.name == 'nt' else 'clear')
        log = buf.getvalue()
        for s in log:
            print(s, end="")
    except SerialException:
        print("Disconnected")
