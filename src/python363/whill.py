#####################################################################################
# WHILL Control Package for Python
# Requirements: Python 3.6.3, pyserial
# Copyright (C) 2018 WHILL, Inc.
#####################################################################################

import serial
from enum import IntEnum, auto


class ComWHILL(serial.Serial):

    class CommandID(IntEnum):
        START = 0
        STOP = auto()
        SET_POWER = auto()
        SET_JOYSTICK = auto()
        SET_SPEED_PROFILE = auto()
        SET_BATTERY_VOLTAGE_OUT = auto()

    class UserControl(IntEnum):
        DISABLE = 0
        ENABLE = auto()

    class PowerCommand(IntEnum):
        OFF = 0
        ON = auto()

    __CMD_LENGTH_TABLE = {
        CommandID.START: 5,
        CommandID.SET_POWER: 2,
        CommandID.SET_JOYSTICK: 4,
        CommandID.SET_SPEED_PROFILE: 11,
        CommandID.SET_BATTERY_VOLTAGE_OUT: 2,
    }

    __PROTOCOL_SIGN = 0xAF

    def __init__(self, port, timeout=None):
        super().__init__(port=port, baudrate=38400, timeout=timeout)

    def recv_data(self):
        received_bytes = self.read_all()
        payload = []
        return payload

    def send_command(self, payload):
        length = self.__CMD_LENGTH_TABLE[payload[0]]
        command_bytes = [self.__PROTOCOL_SIGN, length + 1]
        command_bytes.extend(payload)
        checksum = 0
        for i, x in enumerate(command_bytes):
            if x < 0:
                x = int.from_bytes(x.to_bytes(length=1, byteorder='big', signed=True), byteorder='big', signed=False)
                command_bytes[i] = x
            checksum ^= x
        command_bytes.append(checksum)
        print(command_bytes)
        return self.write(bytes(command_bytes))

    def send_joystick(self, longitudinal=0, lateral=0):
        command_bytes = [self.CommandID.SET_JOYSTICK, self.UserControl.DISABLE, longitudinal, lateral]
        return self.send_command(command_bytes)

    def send_stop(self):
        return self.send_joystick()

    def release_joystick(self):
        command_bytes = [self.CommandID.SET_JOYSTICK, self.UserControl.ENABLE, 0, 0]
        return self.send_command(command_bytes)

    def start_data_stream(self, interval_msec, data_set_num, speed_mode):
        command_bytes = [self.CommandID.START, data_set_num, interval_msec >> 8, interval_msec & 0xFF, speed_mode]
        return self.send_command(command_bytes)

    def stop_data_stream(self):
        command_bytes = [self.CommandID.STOP]
        return self.send_command(command_bytes)

    def send_power_on(self):
        command_bytes = [self.CommandID.SET_POWER, self.PowerCommand.ON]
        return self.send_command(command_bytes)

    def send_power_off(self):
        command_bytes = [self.CommandID.SET_POWER, self.PowerCommand.OFF]
        return self.send_command(command_bytes)

    def set_speed_profile(self, profile_id,
                          forward_speed, forward_accel, forward_decel,
                          backward_speed, backward_accel, backward_decel,
                          turn_speed, turn_accel, turn_decel):
        command_bytes = [self.CommandID.SET_SPEED_PROFILE,
                         profile_id,
                         forward_speed, forward_accel, forward_decel,
                         backward_speed, backward_accel, backward_decel,
                         turn_speed, turn_accel, turn_decel]
        return self.send_command(command_bytes)

    def set_battery_voltage_output_mode(self, vbatt_on_off):
        command_bytes = [self.CommandID.SET_BATTERY_VOLTAGE_OUT, vbatt_on_off]
        return self.send_command(command_bytes)
