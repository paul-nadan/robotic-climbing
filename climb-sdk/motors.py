"""
Interface class for simultaneously controlling multiple Dynamixel motors
"""

from dynamixel_sdk import *
# import roslibpy        # pip install roslibpy, pip install service_identity

# Supported motor series (with protocol version)
SERIES = {"AX": 1, "XM": 2}
# Supported motor models (with speed in RPM, stall torque in Nm)
MODELS = {"AX12-A": (59, 1.5), "AX18-A": (97, 1.8), "XM430-W210-T": (77, 3.0), "XM430-W350-T": (46, 4.1)}

# Control table AX
GOAL_POSITION_AX = 30, 2
PRESENT_POSITION_AX = 36, 2
TORQUE_LIMIT_AX = 34, 2
PRESENT_LOAD_AX = 40, 2
MOVING_SPEED_AX = 32, 2
PRESENT_SPEED_AX = 38, 2
PRESENT_VOLTAGE_AX = 42, 1
PRESENT_TEMPERATURE_AX = 43, 1
TORQUE_ENABLE_AX = 24, 1
CW_ANGLE_LIMIT_AX = 6, 2
CCW_ANGLE_LIMIT_AX = 8, 2

# Control table XM
GOAL_POSITION_XM = 116, 4
PRESENT_POSITION_XM = 132, 4
GOAL_CURRENT_XM = 102, 2
PRESENT_CURRENT_XM = 126, 2
PROFILE_VELOCITY_XM = 112, 4
PRESENT_VELOCITY_XM = 128, 4
GOAL_VELOCITY_XM = 104, 4
PRESENT_INPUT_VOLTAGE_XM = 144, 2
PRESENT_TEMPERATURE_XM = 146, 1
OPERATING_MODE_XM = 11, 1
TORQUE_ENABLE_XM = 64, 1


class Motors:

    def __init__(self, port='/dev/ttyUSB0', baud=1000000):
        """
        Initialize the Dynamixel port

        :param port: USB port used for the Dynamixel interface
        :param baud: Baud rate in Hz
        """
        self.motors_by_id = {}
        self.motors_by_series = {series: [] for series in SERIES}
        self.portHandler = PortHandler(port)
        self.packetHandler1 = PacketHandler(1.0)
        self.packetHandler2 = PacketHandler(2.0)
        self.opened = False
        self.status = "Disconnected"
        self.baud = baud
        self.connect()
        # ros = roslibpy.Ros(host='localhost', port=9090)
        # print(ros.is_connected)
        # ros.run()

    def connect(self):
        """
        Attempt to open a connection to the Dynamixel interface
        """
        self.opened = False
        try:
            self.portHandler.openPort()
            self.portHandler.setBaudRate(self.baud)
            self.opened = True
            self.status = "Connected"
        except Exception as e:
            print(e)
        return self.opened

    def add(self, ids, model, lower=float('-inf'), upper=float('inf'), mirror=(), offset=None):
        """
        Add motors to the robot

        :param ids: List of motor id numbers (or int for single motor)
        :param model: Motor model string (e.g. "XM430-W350-T")
        :param lower: Lower angle limit in deg
        :param upper: Upper angle limit in deg
        :param mirror: List of mirrored motor id numbers (or bool for single motor)
        :param offset: Dictionary mapping motor id to offset angle in deg (or float for single motor)
        """
        if not hasattr(ids, '__iter__'):
            if offset:
                offset = {ids: offset}
            mirror = (ids,) if mirror else ()
            ids = (ids,)
        series = model[0:2].upper()
        full_model = None
        added = []
        for m in MODELS:
            if model.upper() == m[0:len(model)]:
                full_model = m
                break
        if not full_model:
            if series in SERIES:
                print("Unsupported motor model " + model.upper() + ", using default " + series + " model")
                full_model = series
            else:
                print("Unsupported motor series " + series + ", motors could not be added")
                return
        if not offset:
            offset = {}
        for id in ids:
            o = offset[id] if id in offset else 0
            motor = Motor(id, full_model, lower, upper, id in mirror, SERIES[series], *MODELS[full_model], o)
            self.motors_by_series[series].append(motor)
            self.motors_by_id[id] = motor
            added.append(motor)
        return added

    def get(self, ids=None):
        """
        Access a specific motor or list of motors by id number

        :param ids: Motor id number or list of id numbers to access (defaults to all motors)
        :returns: Motor object or list of motor objects
        """
        if hasattr(ids, '__iter__'):
            motor_list = []
            for id in ids:
                if id in self.motors_by_id:
                    motor_list.append(self.motors_by_id[id])
            return motor_list
        else:
            if not ids:
                return list(self.motors_by_id.values())
            if ids in self.motors_by_id:
                return self.motors_by_id[ids]

    def write(self, ids, address, length, value):
        """
        Write a raw value to target motors

        :param ids: Motor id(s)
        :param address: Control table address
        :param length: Number of bytes
        :param value: Value to write
        :return:
        """
        if not self.opened:
            return
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for id in ids:
            if self.motors_by_id[id].protocol == 1:
                self.packetHandler1.writeTxRx(self.portHandler, id, address, length, float2bytes(value, length))
            else:
                self.packetHandler2.writeTxRx(self.portHandler, id, address, length, float2bytes(value, length))
            time.sleep(.001)

    def enable(self, ids=None, velocity_mode=False):
        """
        Torque enable motors and reset operating mode

        :param ids: Motor id number(s) to enable (defaults to all)
        :param velocity_mode: Determines if motor is enabled in position or velocity mode
        """
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for series, motor_list in self.motors_by_series.items():
            motor_list = list(filter(lambda m: m.id in ids or ids[0] is None, motor_list))
            if series == "AX":
                for motor in motor_list:
                    self.write(motor.id, *TORQUE_ENABLE_AX, 0)
                    if velocity_mode:
                        self.write(motor.id, *CW_ANGLE_LIMIT_AX, 0)
                        self.write(motor.id, *CCW_ANGLE_LIMIT_AX, 0)
                    else:
                        self.write(motor.id, *CW_ANGLE_LIMIT_AX, 0)
                        self.write(motor.id, *CCW_ANGLE_LIMIT_AX, 1023)
                    self.write(motor.id, *TORQUE_ENABLE_AX, 1)
                    motor.velocity_mode = velocity_mode
            elif series == "XM":
                for motor in motor_list:
                    self.write(motor.id, *TORQUE_ENABLE_XM, 0)
                    if velocity_mode:  # Velocity control mode
                        self.write(motor.id, *OPERATING_MODE_XM, 1)
                    else:  # Current-based position control mode
                        self.write(motor.id, *OPERATING_MODE_XM, 5)
                    self.write(motor.id, *TORQUE_ENABLE_XM, 1)
                    motor.velocity_mode = velocity_mode

    def disable(self, ids=None):
        """
        Torque disable motors

        :param ids: Motor id number(s) to enable (default is all motors)
        """
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for series, motor_list in self.motors_by_series.items():
            motor_list = list(filter(lambda m: m.id in ids or ids[0] is None, motor_list))
            if series == "AX":
                for motor in motor_list:
                    self.write(motor.id, *TORQUE_ENABLE_AX, 0)
            elif series == "XM":
                for motor in motor_list:
                    self.write(motor.id, *TORQUE_ENABLE_XM, 0)

    def read_angle(self, ids=None):
        """
        Read current motor angles in degrees
        """
        if not self.opened:  # Assume motors have reached their setpoints
            for motor in self.get(ids):
                motor.angle = motor.set_angle
            return
        self.status = "No Motors"
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for series, motor_list in self.motors_by_series.items():
            motor_list = list(filter(lambda m: m.id in ids or ids[0] is None, motor_list))
            if series == "AX":
                for motor in motor_list:
                    raw, _, _ = self.packetHandler1.readTxRx(self.portHandler, motor.id, *PRESENT_POSITION_AX)
                    if len(raw):
                        self.status = "Connected"
                    motor.angle = (bytes2float(raw) / 1023.0 * 300 - 150) * motor.mirror - motor.offset
            elif series == "XM":
                sync = GroupSyncRead(self.portHandler, self.packetHandler2, *PRESENT_POSITION_XM)
                for motor in motor_list:
                    sync.addParam(motor.id)
                sync.txRxPacket()
                for motor in motor_list:
                    raw = sync.getData(motor.id, *PRESENT_POSITION_XM)
                    if raw:
                        self.status = "Connected"
                    motor.angle = (raw / 4095.0 * 360 - 180) * motor.mirror - motor.offset
            else:
                print("read_angle not implemented yet for " + series + "series")
                continue

    def write_angle(self, ids=None):
        """
        Write desired motor angles in degrees
        """
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for series, motor_list in self.motors_by_series.items():
            motor_list = list(filter(lambda m: m.id in ids or ids[0] is None, motor_list))
            for motor in motor_list:
                if motor.set_angle < motor.lower:
                    motor.set_angle = motor.lower
                if motor.set_angle > motor.upper:
                    motor.set_angle = motor.upper
                if motor.goal_angle < motor.lower:
                    motor.goal_angle = motor.lower
                if motor.goal_angle > motor.upper:
                    motor.goal_angle = motor.upper
            if series == "AX":
                sync = GroupSyncWrite(self.portHandler, self.packetHandler1, *GOAL_POSITION_AX)
                for motor in motor_list:
                    raw = ((motor.set_angle + motor.offset) * motor.mirror + 150) * 1023.0 / 300.0
                    sync.addParam(motor.id, float2bytes(raw, sync.data_length))
                if self.opened:
                    sync.txPacket()
            elif series == "XM":
                sync = GroupSyncWrite(self.portHandler, self.packetHandler2, *GOAL_POSITION_XM)
                for motor in motor_list:
                    raw = ((motor.set_angle + motor.offset) * motor.mirror + 180) * 4095.0 / 360.0
                    sync.addParam(motor.id, float2bytes(raw, sync.data_length))
                if self.opened:
                    sync.txPacket()
            else:
                print("write_angle not implemented yet for " + series + "series")
                continue

    def read_velocity(self, ids=None):
        """
        Read current motor velocity in degrees per second
        """
        if not self.opened:
            return
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for series, motor_list in self.motors_by_series.items():
            motor_list = list(filter(lambda m: m.id in ids or ids[0] is None, motor_list))
            if series == "AX":
                for motor in motor_list:
                    raw, _, _ = self.packetHandler1.readTxRx(self.portHandler, motor.id, *PRESENT_SPEED_AX)
                    raw = bytes2float(raw)
                    if raw >= 1024:
                        raw = -(raw - 1024)
                    motor.velocity = raw * 0.111 * 6 * motor.mirror
            elif series == "XM":
                sync = GroupSyncRead(self.portHandler, self.packetHandler2, *PRESENT_VELOCITY_XM)
                for motor in motor_list:
                    sync.addParam(motor.id)
                sync.txRxPacket()
                for motor in motor_list:
                    raw = sync.getData(motor.id, *PRESENT_VELOCITY_XM)
                    if raw >= 32768:
                        raw -= 32768 * 2
                    motor.velocity = raw * 0.229 * 6 * motor.mirror
            else:
                print("read_velocity not implemented yet for " + series + "series")
                continue

    def write_velocity(self, ids=None):
        """
        Write desired motor velocity or moving speed (depending on motor operating mode) in degrees per second
        """
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for series, motor_list in self.motors_by_series.items():
            motor_list = list(filter(lambda m: m.id in ids or ids[0] is None, motor_list))
            if series == "AX":
                sync = GroupSyncWrite(self.portHandler, self.packetHandler1, *MOVING_SPEED_AX)
                for motor in motor_list:
                    raw = motor.set_velocity / 0.111 / 6
                    if motor.velocity_mode:
                        raw = abs(motor.set_velocity) / motor.speed * 1023
                        if motor.set_velocity < 0:
                            raw += 1024
                    sync.addParam(motor.id, float2bytes(raw, sync.data_length))
                if self.opened:
                    sync.txPacket()
            elif series == "XM":
                sync_vel = GroupSyncWrite(self.portHandler, self.packetHandler2, *GOAL_VELOCITY_XM)
                sync = GroupSyncWrite(self.portHandler, self.packetHandler2, *PROFILE_VELOCITY_XM)
                for motor in motor_list:
                    raw = motor.set_velocity / 0.229 / 6
                    if not motor.velocity_mode:
                        sync_vel.addParam(motor.id, float2bytes(raw, sync.data_length))
                    else:
                        sync.addParam(motor.id, float2bytes(raw, sync.data_length))
                if sync_vel.is_param_changed and self.opened:
                    sync_vel.txPacket()
                if sync.is_param_changed and self.opened:
                    sync.txPacket()
            else:
                print("write_velocity not implemented yet for " + series + "series")
                continue

    def read_torque(self, ids=None):
        """
        Read current motor torque in Newton millimeters
        """
        if not self.opened:
            return
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for series, motor_list in self.motors_by_series.items():
            motor_list = list(filter(lambda m: m.id in ids or ids[0] is None, motor_list))
            if series == "AX":
                for motor in motor_list:
                    raw, _, _ = self.packetHandler1.readTxRx(self.portHandler, motor.id, *PRESENT_LOAD_AX)
                    raw = bytes2float(raw)
                    if raw >= 1024:
                        raw = -(raw - 1024)
                    torque = -raw / 1023.0 * motor.stall * motor.mirror
                    motor.torque_list = (*motor.torque_list[1:], torque)
                    motor.torque = sum(motor.torque_list)/len(motor.torque_list)
            elif series == "XM":
                sync = GroupSyncRead(self.portHandler, self.packetHandler2, *PRESENT_CURRENT_XM)
                for motor in motor_list:
                    sync.addParam(motor.id)
                sync.txRxPacket()
                for motor in motor_list:
                    raw = sync.getData(motor.id, *PRESENT_CURRENT_XM)
                    if raw >= 32768:
                        raw -= 32768 * 2
                    torque = raw * 2.69 * motor.stall / 2300 * motor.mirror
                    motor.torque_list = (*motor.torque_list[1:], torque)
                    motor.torque = sum(motor.torque_list)/len(motor.torque_list)
            else:
                print("read_torque not implemented yet for " + series + "series")
                continue

    def write_torque(self, ids=None):
        """
        Write desired motor torque in Newton millimeters
        """
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for series, motor_list in self.motors_by_series.items():
            motor_list = list(filter(lambda m: m.id in ids or ids[0] is None, motor_list))
            if series == "AX":
                sync = GroupSyncWrite(self.portHandler, self.packetHandler1, *TORQUE_LIMIT_AX)
                for motor in motor_list:
                    raw = min(abs(motor.set_torque), motor.stall) / motor.stall * 1023
                    sync.addParam(motor.id, float2bytes(raw, sync.data_length))
                if self.opened:
                    sync.txPacket()
            elif series == "XM":
                sync = GroupSyncWrite(self.portHandler, self.packetHandler2, *GOAL_CURRENT_XM)
                for motor in motor_list:
                    raw = min(abs(motor.set_torque), motor.stall) * 2300 / motor.stall / 2.69
                    # raw = abs(motor.set_torque) * 2300 / motor.stall / 2.69
                    sync.addParam(motor.id, float2bytes(raw, sync.data_length))
                if self.opened:
                    sync.txPacket()
            else:
                print("write_torque not implemented yet for " + series + "series")
                continue

    def read_voltage(self, ids=None):
        """
        Read current motor voltage in volts
        """
        if not self.opened:
            return
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for series, motor_list in self.motors_by_series.items():
            motor_list = list(filter(lambda m: m.id in ids or ids[0] is None, motor_list))
            if series == "AX":
                for motor in motor_list:
                    raw, _, _ = self.packetHandler1.readTxRx(self.portHandler, motor.id, *PRESENT_VOLTAGE_AX)
                    motor.voltage = bytes2float(raw) / 10.0
            elif series == "XM":
                sync = GroupSyncRead(self.portHandler, self.packetHandler2, *PRESENT_INPUT_VOLTAGE_XM)
                for motor in motor_list:
                    sync.addParam(motor.id)
                sync.txRxPacket()
                for motor in motor_list:
                    raw = sync.getData(motor.id, *PRESENT_INPUT_VOLTAGE_XM)
                    motor.voltage = raw / 10.0
            else:
                print("read_voltage not implemented yet for " + series + "series")
                continue

    def read_temp(self, ids=None):
        """
        Read current motor temperature in Celsius
        """
        if not self.opened:
            return
        ids = (ids,) if not hasattr(ids, '__iter__') else ids
        for series, motor_list in self.motors_by_series.items():
            motor_list = list(filter(lambda m: m.id in ids or ids[0] is None, motor_list))
            if series == "AX":
                for motor in motor_list:
                    raw, _, _ = self.packetHandler1.readTxRx(self.portHandler, motor.id, *PRESENT_TEMPERATURE_AX)
                    motor.temperature = bytes2float(raw)
            elif series == "XM":
                sync = GroupSyncRead(self.portHandler, self.packetHandler2, *PRESENT_TEMPERATURE_XM)
                for motor in motor_list:
                    sync.addParam(motor.id)
                sync.txRxPacket()
                for motor in motor_list:
                    raw = sync.getData(motor.id, *PRESENT_TEMPERATURE_XM)
                    motor.temperature = raw
            else:
                print("read_voltage not implemented yet for " + series + "series")
                continue


class Motor:
    def __init__(self, id, model, lower, upper, mirror, protocol, speed, stall, offset, average_torque=3):
        """
        Create a Dynamixel motor object

        :param id: Motor id number
        :param model: Motor model string (e.g. "XM430-W350-T")
        :param lower: Motor lower angle limit in degrees
        :param upper: Motor upper angle limit in degrees
        :param mirror: True if motor direction should be reversed
        :param protocol: Either 1 or 2
        :param speed: Motor no-load speed in RPM
        :param stall: Motor stall torque in Newton meters
        :param offset: Motor angle offset in degrees
        :param average_torque: Number of previous torque measurements to average together
        """
        self.id = id                            # motor id number
        self.model = model                      # motor model string
        self.lower = lower                      # motor lower angle limit (deg)
        self.upper = upper                      # motor upper angle limit (deg)
        self.mirror = -1 if mirror else 1       # sign of motor direction (1 or -1)
        self.stall = stall*1000                 # stall torque (Nmm)
        self.speed = speed*6                    # no load speed (deg/s)
        self.protocol = protocol                # communication protocol (1 or 2)
        self.offset = offset                    # angle offset (deg)
        self.angle = 0                          # angle read from motor (deg)
        self.set_angle = 0                      # angle to write to motor (deg)
        self.goal_angle = self.set_angle        # angle setpoint (deg)
        self.velocity = 0                       # velocity read from motor (deg/s)
        self.set_velocity = 0                   # velocity to write to motor (deg/s)
        self.goal_velocity = self.set_velocity  # velocity setpoint (deg/s)
        self.torque = 0                         # torque read from motor (Nmm)
        self.set_torque = self.stall            # torque to write to motor (Nmm)
        self.goal_torque = self.set_torque      # torque setpoint (Nmm)
        self.velocity_mode = False              # control motor velocity instead of motor angle
        self.voltage = 0                        # motor input voltage (V)
        self.temperature = 0                    # motor internal temperature (deg C)
        self.torque_list = (0.0, ) * average_torque    # List of the last average_torque torques


def float2bytes(value, n_bytes):
    """
    Convert float to bytes for Dynamixel write
    :param value: Original float
    :param n_bytes: Number of bytes in array
    :return: Tuple of bytes
    """
    value = int(value)
    if n_bytes == 1:
        return value,
    elif n_bytes == 2:
        return DXL_LOBYTE(value), DXL_HIBYTE(value)
    elif n_bytes == 4:
        return DXL_LOBYTE(DXL_LOWORD(value)), DXL_HIBYTE(DXL_LOWORD(value)), \
               DXL_LOBYTE(DXL_HIWORD(value)), DXL_HIBYTE(DXL_HIWORD(value))
    else:
        print("Invalid n_bytes value: " + n_bytes)


def bytes2float(raw):
    """
    Convert bytes to float for Dynamixel read
    :param raw: Byte array of length 1, 2 or 4
    :return: Float value
    """
    if len(raw) == 1:
        return raw[0]
    elif len(raw) == 2:
        return DXL_MAKEWORD(*raw)
    elif len(raw) == 4:
        return DXL_MAKEDWORD(DXL_MAKEWORD(raw[0], raw[1]), DXL_MAKEWORD(raw[2], raw[3]))
    else:
        return 0


# Example usage
if __name__ == '__main__':

    # Setup
    motors = Motors(port='COM8', baud=1000000)  # Initialize Serial port
    motors.add([1, 2, 3, 4], 'XM430-W350-T', lower=-50, upper=50)  # Add 4 XM series motors
    motors.add([5, 6, 7, 8], 'AX18-A', lower=-70, upper=70, mirror=(7, 8))  # Add 4 AX series motors
    motors.enable(velocity_mode=True)  # Enable torque on all motors

    # Read
    motors.read_velocity(ids=7)  # Read current motor velocity for 1 motor
    print(motors.get(7).velocity)  # Print the measured motor velocity

    # Write
    motors.get(2).set_angle = 50  # Set desired motor angle
    motors.get(6).set_angle = 20  # Set desired motor angle
    motors.write_angle(ids=(2, 6))  # Write desired motor angles for 2 motors
