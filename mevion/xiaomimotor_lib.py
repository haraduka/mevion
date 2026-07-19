import can
import copy
import struct
import math

legitimate_motors = [
                    "CyberGear",
                    "RobStride00",
                    "RobStride01",
                    "RobStride02",
                    "RobStride03",
                    "RobStride04",
                    "RobStride05",
                    ]

CyberGear_PARAMS = {
        "P_MIN" : -4 * math.pi,
        "P_MAX" : 4 * math.pi,
        "V_MIN" : -30.0,
        "V_MAX" : 30.0,
        "KP_MIN" : 0.0,
        "KP_MAX" : 500.0,
        "KD_MIN" : 0.0,
        "KD_MAX" : 5.0,
        "T_MIN" : -12.0,
        "T_MAX" : 12.0,
        "AXIS_DIRECTION" : 1
        }

RobStride00_PARAMS = {
        "P_MIN" : -4 * math.pi,
        "P_MAX" : 4 * math.pi,
        "V_MIN" : -33.0,
        "V_MAX" : 33.0,
        "KP_MIN" : 0.0,
        "KP_MAX" : 500.0,
        "KD_MIN" : 0.0,
        "KD_MAX" : 5.0,
        "T_MIN" : -14.0,
        "T_MAX" : 14.0,
        "AXIS_DIRECTION" : 1
        }

RobStride01_PARAMS = { # the same with RobStride02
        "P_MIN" : -4 * math.pi,
        "P_MAX" : 4 * math.pi,
        "V_MIN" : -44.0,
        "V_MAX" : 44.0,
        "KP_MIN" : 0.0,
        "KP_MAX" : 500.0,
        "KD_MIN" : 0.0,
        "KD_MAX" : 5.0,
        "T_MIN" : -17.0,
        "T_MAX" : 17.0,
        "AXIS_DIRECTION" : 1
        }

RobStride03_PARAMS = {
        "P_MIN" : -4 * math.pi,
        "P_MAX" : 4 * math.pi,
        "V_MIN" : -20.0,
        "V_MAX" : 20.0,
        "KP_MIN" : 0.0,
        "KP_MAX" : 5000.0,
        "KD_MIN" : 0.0,
        "KD_MAX" : 100.0,
        "T_MIN" : -60.0,
        "T_MAX" : 60.0,
        "AXIS_DIRECTION" : 1
        }

RobStride04_PARAMS = {
        "P_MIN" : -4 * math.pi,
        "P_MAX" : 4 * math.pi,
        "V_MIN" : -15.0,
        "V_MAX" : 15.0,
        "KP_MIN" : 0.0,
        "KP_MAX" : 500.0,
        "KD_MIN" : 0.0,
        "KD_MAX" : 5.0,
        "T_MIN" : -120.0,
        "T_MAX" : 120.0,
        "AXIS_DIRECTION" : 1
        }

RobStride05_PARAMS = {
        "P_MIN" : -4 * math.pi,
        "P_MAX" : 4 * math.pi,
        "V_MIN" : -50.0,
        "V_MAX" : 50.0,
        "KP_MIN" : 0.0,
        "KP_MAX" : 500.0,
        "KD_MIN" : 0.0,
        "KD_MAX" : 5.0,
        "T_MIN" : -5.5,
        "T_MAX" : 5.5,
        "AXIS_DIRECTION" : 1
        }

class CanMotorController:
    PARAM_TABLE = {
        "motorOverTemp": {"feature_code": 0x200D, "type": "int16"},
        "overTempTime": {"feature_code": 0x200E, "type": "int32"},
        "limit_torque": {"feature_code": 0x2007, "type": "float"},
        "cur_kp": {"feature_code": 0x2012, "type": "float"},
        "cur_ki": {"feature_code": 0x2013, "type": "float"},
        "spd_kp": {"feature_code": 0x2014, "type": "float"},
        "spd_ki": {"feature_code": 0x2015, "type": "float"},
        "loc_kp": {"feature_code": 0x2016, "type": "float"},
        "spd_filt_gain": {"feature_code": 0x2017, "type": "float"},
        "limit_spd": {"feature_code": 0x2018, "type": "float"},
        "limit_cur": {"feature_code": 0x2019, "type": "float"},
    }

    PARAMETERS = {
        "run_mode": {"index": 0x7005, "format": "u8"},
        "iq_ref": {"index": 0x7006, "format": "f"},
        "spd_ref": {"index": 0x700A, "format": "f"},
        "limit_torque": {"index": 0x700B, "format": "f"},
        "cur_kp": {"index": 0x7010, "format": "f"},
        "cur_ki": {"index": 0x7011, "format": "f"},
        "cur_filt_gain": {"index": 0x7014, "format": "f"},
        "loc_ref": {"index": 0x7016, "format": "f"},
        "limit_spd": {"index": 0x7017, "format": "f"},
        "limit_cur": {"index": 0x7018, "format": "f"},
    }

    COMMAND_MODES = {
        "GET_DEVICE_ID" : 0,
        "MOTOR_CONTROL" : 1,
        "MOTOR_FEEDBACK" : 2,
        "MOTOR_ENABLE" : 3,
        "MOTOR_STOP" : 4,
        "SET_MECHANICAL_ZERO" : 6,
        "SET_MOTOR_CAN_ID" : 7,
        "PARAM_TABLE_WRITE" : 8,
        "SINGLE_PARAM_READ" : 17,
        "SINGLE_PARAM_WRITE" : 18,
        "FAULT_FEEDBACK" : 21
        }

    RUN_MODES = {
        "CONTROL_MODE" : 0,
        "POSITION_MODE" : 1,
        "VELOCITY_MODE" : 2,
        "CURRENT_MODE" : 3
        }

    MOTOR_PARAMS = {
            "P_MIN" : -4 * math.pi,
            "P_MAX" : 4 * math.pi,
            "V_MIN" : -30.0,
            "V_MAX" : 30.0,
            "KP_MIN" : 0.0,
            "KP_MAX" : 500.0,
            "KD_MIN" : 0.0,
            "KD_MAX" : 5.0,
            "T_MIN" : -12.0,
            "T_MAX" : 12.0,
            "AXIS_DIRECTION" : 1
            }

    bus = {}

    def __init__(self, bus="can0", motor_id=127, main_can_id=254, motor_dir=1, motor_type="CyberGear"):
        self.bus_name = bus
        self.motor_id = motor_id
        self.main_can_id = main_can_id
        self.motorParams = CyberGear_PARAMS
        print(f"Using Motor Type: {motor_type} (id: {motor_id})")
        assert motor_type in legitimate_motors, "Motor Type not in list of accepted motors."
        if motor_type == "CyberGear":
            self.motorParams = copy.deepcopy(CyberGear_PARAMS)
        elif motor_type == "RobStride00":
            self.motorParams = copy.deepcopy(RobStride00_PARAMS)
        elif motor_type == "RobStride01":
            self.motorParams = copy.deepcopy(RobStride01_PARAMS)
        elif motor_type == "RobStride02":
            self.motorParams = copy.deepcopy(RobStride01_PARAMS)
        elif motor_type == "RobStride03":
            self.motorParams = copy.deepcopy(RobStride03_PARAMS)
        elif motor_type == "RobStride04":
            self.motorParams = copy.deepcopy(RobStride04_PARAMS)
        elif motor_type == "RobStride05":
            self.motorParams = copy.deepcopy(RobStride05_PARAMS)
        self.motorParams["AXIS_DIRECTION"] = motor_dir
        self.angle_range = None # [rad]
        self.angle_offset = None # [rad]
        self.angle_scale = 1.0

        self.pos_cur = 0 # [rad]
        self.vel_cur = 0 # [rad/s]
        self.tau_cur = 0 # [Nm]
        self.temp_cur = 20 # [C]

        if bus in CanMotorController.bus:
            print("CanBus already available")
        else:
            try:
                CanMotorController.bus[bus] = can.interface.Bus(interface="socketcan", channel=bus, bitrate=1000000)
            except Exception as e:
                print("Error:", e)

        self.disable_motor()
        # self.set_zero_pos()

    def _float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        x = max(min(x, x_max), x_min) # Clamp x to the range [x_min, x_max]
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    def _uint_to_float(self, x, x_min, x_max, bits):
        span = (1 << bits) - 1
        offset = x_max - x_min
        x = max(min(x, span), 0) # Clamp x to the range [0, span]
        return offset * x / span + x_min

    def _linear_mapping(self, value, value_min, value_max, target_min=0, target_max=65535):
       return int((value - value_min) / (value_max - value_min) * (target_max - target_min) + target_min)

    def set_angle_range(self, low, upper, deg=False):
        if deg:
            self.angle_range = [low*math.pi/180, upper*math.pi/180]
        else:
            self.angle_range = [low, upper]
        assert len(self.angle_range) == 2, "Invalid Angle Range Specified."
        assert self.angle_range[0] < self.angle_range[1], "Invalid Angle Range Specified."

    def set_angle_offset(self, angle_offset, deg=False):
        if deg:
            self.angle_offset = angle_offset*math.pi/180
        else:
            self.angle_offset = angle_offset

    def set_angle_scale(self, angle_scale):
        self.angle_scale = angle_scale

    def decode_data(self, data=[], format="f f"):
        format_list = format.split()
        rdata = []
        for i in range(len(format_list)):
            f = format_list[i]
            s_f = []
            if f == "f":
                s_f = [4, "f"]
            elif f == "u16":
                s_f = [2, "H"]
            elif f == "s16":
                s_f = [2, "h"]
            elif f == "u32":
                s_f = [4, "I"]
            elif f == "s32":
                s_f = [4, "i"]
            elif f == "u8":
                s_f = [1, "B"]
            elif f == "s8":
                s_f = [1, "b"]
            if f != "f":
                data[i] = int(data[i])
            if len(s_f) == 2:
                bs = struct.pack(s_f[1], data[i])
                for j in range(s_f[0]):
                    rdata.append(bs[j])
            else:
                print("unkown format in format_data(): " + f)
                return []
        if len(rdata) < 4:
            for i in range(4 - len(rdata)):
                rdata.append(0x00)
        return rdata

    def clear_can_rx(self, timeout, max_attempts):
        timeout_seconds = timeout / 1000.0
        attempts = 0
        while attempts < max_attempts:
            received_msg = CanMotorController.bus[self.bus_name].recv(timeout=timeout_seconds)
            if received_msg is None:
                break
            attempts += 1

    def clear_can_rx(self):
        while True:
            received_msg = CanMotorController.bus[self.bus_name].recv(timeout=0)
            if received_msg is None:
                break

    def write_single_param(self, param_name, value):
        param_info = self.PARAMETERS.get(param_name)
        if param_info is None:
            print(f"Unknown parameter name: {param_name}")
            return
        index = param_info["index"]
        format = param_info["format"]

        encoded_data = self.decode_data(data=[value], format=format)
        data1 = [b for b in struct.pack("<I", index)] + encoded_data

        self.clear_can_rx()

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.COMMAND_MODES["SINGLE_PARAM_WRITE"],
            data2=self.main_can_id,
            data1=data1,
            timeout=2000,
        )
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def send_receive_can_message(self, cmd_mode, data2, data1, timeout=1000):
        timeout_seconds = timeout / 1000.0
        # Calculate the arbitration ID
        arbitration_id = (cmd_mode << 24) | (data2 << 8) | self.motor_id
        message = can.Message(arbitration_id=arbitration_id, data=data1, is_extended_id=True)

        # Send the CAN message
        try:
            CanMotorController.bus[self.bus_name].send(message)
            # print(f"Sent message with ID {hex(arbitration_id)}, data: {data1}")
        except:
            print("Failed to send the message.")
            return None, None

        # Receive a CAN message with a 1-second timeout
        received_msg = CanMotorController.bus[self.bus_name].recv(timeout=timeout_seconds)
        if received_msg:
            motor_can_id = (received_msg.arbitration_id >> 8) & 0xFF
            if self.motor_id != motor_can_id:
                received_msg = CanMotorController.bus[self.bus_name].recv(timeout=timeout_seconds)
                if received_msg:
                    return received_msg.data, received_msg.arbitration_id
                else:
                    return None, None
            return received_msg.data, received_msg.arbitration_id
        else:
            return None, None

    def send_can_message(self, cmd_mode, data2, data1):
        # Calculate the arbitration ID
        arbitration_id = (cmd_mode << 24) | (data2 << 8) | self.motor_id
        message = can.Message(arbitration_id=arbitration_id, data=data1, is_extended_id=True)

        # Send the CAN message
        try:
            CanMotorController.bus[self.bus_name].send(message)
            # print(f"Sent message with ID {hex(arbitration_id)}, data: {data1}")
        except:
            print("Failed to send the message.")
            return False
        return True

    def receive_can_raw_message(self, timeout=1000):
        timeout_seconds = timeout / 1000.0
        # Receive a CAN message with a 1-second timeout
        received_msg = CanMotorController.bus[self.bus_name].recv(timeout=timeout_seconds)
        if received_msg:
            arbitration_id = received_msg.arbitration_id
            motor_can_id = (arbitration_id >> 8) & 0xFF
            return motor_can_id, received_msg.data, arbitration_id
        else:
            return None, None, None

    def parse_received_msg(self, data, arbitration_id):
        if data is not None:
            motor_can_id = (arbitration_id >> 8) & 0xFF
            error_code = (arbitration_id >> 16) & 0x3F
            TWO_BYTES_BITS = 16
            pos = self._uint_to_float((data[0] << 8) + data[1], self.motorParams["P_MIN"], self.motorParams["P_MAX"], TWO_BYTES_BITS) * self.motorParams["AXIS_DIRECTION"]
            vel = self._uint_to_float((data[2] << 8) + data[3], self.motorParams["V_MIN"], self.motorParams["V_MAX"], TWO_BYTES_BITS) * self.motorParams["AXIS_DIRECTION"]
            tau = self._uint_to_float((data[4] << 8) + data[5], self.motorParams["T_MIN"], self.motorParams["T_MAX"], TWO_BYTES_BITS) * self.motorParams["AXIS_DIRECTION"]
            temp_raw = (data[6] << 8) + data[7]
            temp = temp_raw / 10.0

            if self.angle_offset is not None:
                pos = pos + self.angle_offset
            self.pos_cur = pos*self.angle_scale # [rad] * [m/rad] = [m]
            self.vel_cur = vel*self.angle_scale # [rad/s] * [m/rad] = [m/s]
            self.tau_cur = tau/self.angle_scale # [Nm] * [rad/m] = [N]
            self.temp_cur = temp
            return motor_can_id, error_code, self.pos_cur, self.vel_cur, self.tau_cur, self.temp_cur
        else:
            print("No message received within the timeout period.")
            return None, None, None, None, None, None

    def enable_motor(self):
        self.clear_can_rx()

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.COMMAND_MODES["MOTOR_ENABLE"],
            data2=self.main_can_id,
            data1=[],
            timeout=2000,
        )
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def disable_motor(self):
        self.clear_can_rx()

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.COMMAND_MODES["MOTOR_STOP"],
            data2=self.main_can_id,
            data1=[0, 0, 0, 0, 0, 0, 0, 0],
        )
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def set_run_mode(self, mode):
        if not mode in self.RUN_MODES:
            raise ValueError(f"Invalid mode: {mode}. Must be an instance of RUN_MODES")
        return self.write_single_param("run_mode", value=self.RUN_MODES[mode])

    def set_zero_pos(self):
        self.clear_can_rx()

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.COMMAND_MODES["SET_MECHANICAL_ZERO"],
            data2=self.main_can_id,
            data1=[1],
            timeout=2000,
        )
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def change_motor_id(self, new_motor_id):
        self.clear_can_rx()

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.COMMAND_MODES["SET_MOTOR_CAN_ID"],
            data2=(new_motor_id << 8) | self.main_can_id,
            data1=[0, 0, 0, 0, 0, 0, 0 ,0],
        )
        self.motor_id = new_motor_id
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def send_control_command(self, p_ref, v_ref, kp, kd, tau_ff):
        if self.angle_range is not None:
            p_ref = max(self.angle_range[0], min(p_ref, self.angle_range[1]))
        p_ref = p_ref / self.angle_scale
        v_ref = v_ref / self.angle_scale
        tau_ff = tau_ff * self.angle_scale
        if self.angle_offset is not None:
            p_ref = p_ref - self.angle_offset
        p_ref = min(max(self.motorParams["P_MIN"], p_ref), self.motorParams["P_MAX"])
        v_ref = min(max(self.motorParams["V_MIN"], v_ref), self.motorParams["V_MAX"])
        kp = min(max(self.motorParams["KP_MIN"], kp), self.motorParams["KP_MAX"])
        kd = min(max(self.motorParams["KD_MIN"], kd), self.motorParams["KD_MAX"])
        tau_ff = max(self.motorParams["T_MIN"], min(tau_ff, self.motorParams["T_MAX"]))

        p_ref_mapped = self._linear_mapping(p_ref * self.motorParams["AXIS_DIRECTION"], self.motorParams["P_MIN"], self.motorParams["P_MAX"])
        v_ref_mapped = self._linear_mapping(v_ref * self.motorParams["AXIS_DIRECTION"], self.motorParams["V_MIN"], self.motorParams["V_MAX"])
        kp_mapped = self._linear_mapping(kp, self.motorParams["KP_MIN"], self.motorParams["KP_MAX"])
        kd_mapped = self._linear_mapping(kd, self.motorParams["KD_MIN"], self.motorParams["KD_MAX"])
        tau_ff_mapped = self._linear_mapping(tau_ff * self.motorParams["AXIS_DIRECTION"], self.motorParams["T_MIN"], self.motorParams["T_MAX"])

        data1_bytes = struct.pack(">HHHH", p_ref_mapped, v_ref_mapped, kp_mapped, kd_mapped)
        data1 = [b for b in data1_bytes]
        data2 = tau_ff_mapped

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.COMMAND_MODES["MOTOR_CONTROL"],
            data1=data1,
            data2=data2
        )

        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def only_send_control_command(self, p_ref, v_ref, kp, kd, tau_ff):
        if self.angle_range is not None:
            p_ref = max(self.angle_range[0], min(p_ref, self.angle_range[1]))
        p_ref = p_ref / self.angle_scale
        v_ref = v_ref / self.angle_scale
        tau_ff = tau_ff * self.angle_scale
        if self.angle_offset is not None:
            p_ref = p_ref - self.angle_offset
        p_ref = min(max(self.motorParams["P_MIN"], p_ref), self.motorParams["P_MAX"])
        v_ref = min(max(self.motorParams["V_MIN"], v_ref), self.motorParams["V_MAX"])
        kp = min(max(self.motorParams["KP_MIN"], kp), self.motorParams["KP_MAX"])
        kd = min(max(self.motorParams["KD_MIN"], kd), self.motorParams["KD_MAX"])
        tau_ff = max(self.motorParams["T_MIN"], min(tau_ff, self.motorParams["T_MAX"]))

        p_ref_mapped = self._linear_mapping(p_ref * self.motorParams["AXIS_DIRECTION"], self.motorParams["P_MIN"], self.motorParams["P_MAX"]) 
        v_ref_mapped = self._linear_mapping(v_ref * self.motorParams["AXIS_DIRECTION"], self.motorParams["V_MIN"], self.motorParams["V_MAX"])
        kp_mapped = self._linear_mapping(kp, self.motorParams["KP_MIN"], self.motorParams["KP_MAX"])
        kd_mapped = self._linear_mapping(kd, self.motorParams["KD_MIN"], self.motorParams["KD_MAX"])
        tau_ff_mapped = self._linear_mapping(tau_ff * self.motorParams["AXIS_DIRECTION"], self.motorParams["T_MIN"], self.motorParams["T_MAX"])

        data1_bytes = struct.pack(">HHHH", p_ref_mapped, v_ref_mapped, kp_mapped, kd_mapped)
        data1 = [b for b in data1_bytes]
        data2 = tau_ff_mapped

        return self.send_can_message(cmd_mode=self.COMMAND_MODES["MOTOR_CONTROL"], data1=data1, data2=data2)
