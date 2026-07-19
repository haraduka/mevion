import os
import atexit
import sys
import select
import time
import threading
import subprocess
import weakref
import numpy as np

import skrobot
import pinocchio

from mevion.skrobot_mevion import SkRobotSingleMevion
from mevion.skrobot_mevion import SkRobotDualMevion
from mevion.skrobot_mevion import SkRobotQuadMevion
from mevion.xiaomimotor_lib import CanMotorController
from mevion import parameters as P
from mevion import robot_utils

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
# from mevion.msg import MevionLog

np.set_printoptions(precision=3, suppress=True)
os.environ['PYOPENGL_PLATFORM'] = 'glx'

class RobotState:
    def __init__(self):
        self.angle = []
        self.velocity = []
        self.torque = []
        self.temperature = []
        self.lock = threading.Lock()

class PeripheralState:
    def __init__(self):
        self.spacenav_enable = False
        self.spacenav = [0.0] * 8
        self.spacenav[6] = False # switchable button
        self.spacenav[7] = False # switchable button
        self.virtual_enable = False
        self.virtual = [0.0] * 4
        self.lock = threading.Lock()


class RobotCommand:
    def __init__(self):
        self.angle = []
        self.velocity = []
        self.torque = []
        self.kp = []
        self.kd = []
        self.lock = threading.Lock()

class Mevion(Node):
    _instances = weakref.WeakSet()
    def __init__(self, mode="quad"):
        """
        mode: "single" or "dual" or "quad" (default: dual)
        """
        rclpy.init()
        super().__init__('mevion_utils')
        self.mode = mode
        if self.mode == "single":
            self.params = P.SINGLE_PARAMS
            self.JOINT_NAME = P.JOINT_NAME_SINGLE
            self.LIMB_INDEX = P.LIMB_INDEX_SINGLE
            self.HAND_INDEX = P.HAND_INDEX_SINGLE
        elif self.mode == "dual":
            self.params = P.DUAL_PARAMS
            self.JOINT_NAME = P.JOINT_NAME_DUAL
            self.LIMB_INDEX = P.LIMB_INDEX_DUAL
            self.HAND_INDEX = P.HAND_INDEX_DUAL
        elif self.mode == "quad":
            self.params = P.QUAD_PARAMS
            self.JOINT_NAME = P.JOINT_NAME_QUAD
            self.LIMB_INDEX = P.LIMB_INDEX_QUAD
            self.HAND_INDEX = P.HAND_INDEX_QUAD
        self.N_JOINTS = len(self.JOINT_NAME)

        self.is_skrobot_setup = False
        self.robot_model = None
        self.robot_viewer = None
        self._stop_event = threading.Event()
        self.sim_thread = None

        self.is_pinocchio_setup = False
        self.pinocchio_model = None
        self.pinocchio_data = None
        self.joint_params = None

        self.is_spacenav_setup = False

        self.is_mujoco_setup = False

        self.is_can_setup = False
        self.motors = [None]*self.N_JOINTS

        self.robot_state = RobotState()
        self.peripheral_state = PeripheralState()
        self.robot_command = RobotCommand()

        self.robot_state.angle = [0.0] * self.N_JOINTS
        self.robot_state.velocity = [0.0] * self.N_JOINTS
        self.robot_state.torque = [0.0] * self.N_JOINTS
        self.robot_state.temperature = [0.0] * self.N_JOINTS
        self.robot_command.angle = [0.0] * self.N_JOINTS
        self.robot_command.velocity = [0.0] * self.N_JOINTS
        self.robot_command.torque = [0.0] * self.N_JOINTS
        self.robot_command.kp = self.params["KP_GAIN"][:self.N_JOINTS]
        self.robot_command.kd = self.params["KD_GAIN"][:self.N_JOINTS]

        self._shutdown_lock = threading.Lock()
        self._is_shutdown = False
        atexit.register(self.shutdown)
        self._instances.add(self)

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.spin_background, daemon=True)
        self.spin_thread.start()

    def spin_background(self):
        try:
            self._executor.spin()
        except RuntimeError:
            if not self._is_shutdown:
                raise

    def disable_all_motors(self):
        print("################# Disabling All Motors...")
        for motor in self.motors:
            if motor is not None:
                can_id, error_code, pos, vel, tau, tem = motor.disable_motor()
                print("Disabling Motor {} [Status] Pos: {:.3f}, Vel: {:.3f}, Tau: {:.3f}, Temp: {:.3f}".format(motor.motor_id, pos, vel, tau, tem))

    @classmethod
    def shutdown_all(cls):
        for instance in tuple(cls._instances):
            instance.shutdown()

    def shutdown(self):
        """Stop motors, the ROS executor, and the node exactly once."""
        with self._shutdown_lock:
            if self._is_shutdown:
                return
            self._is_shutdown = True

        self.disable_all_motors()
        self._stop_event.set()
        if (self.sim_thread is not None and self.sim_thread.is_alive() and
                threading.current_thread() is not self.sim_thread):
            self.sim_thread.join(timeout=2.0)
        self._executor.shutdown(timeout_sec=2.0)
        if (self.spin_thread.is_alive() and
                threading.current_thread() is not self.spin_thread):
            self.spin_thread.join(timeout=2.0)
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def setup_sim(self):
        self.setup_skrobot()
        self.setup_pinocchio()
        self.setup_mujoco()

    def setup_real(self):
        self.setup_skrobot()
        self.setup_pinocchio()
        self.setup_can()

    def setup_skrobot(self):
        if self.is_skrobot_setup:
            return
        if self.mode == "single":
            self.robot_model = SkRobotSingleMevion()
        elif self.mode == "dual":
            self.robot_model = SkRobotDualMevion()
        elif self.mode == "quad":
            self.robot_model = SkRobotQuadMevion()
        self.robot_model.reset_pose()

        self.robot_viewer = skrobot.viewers.PyrenderViewer(resolution=(320, 320))
        self.robot_viewer.add(self.robot_model)
        self.robot_viewer.show()
        self.is_skrobot_setup = True

    def setup_pinocchio(self):
        if self.is_pinocchio_setup:
            return
        if self.mode == "single":
            xacro_name = "single_mevion.urdf.xacro"
        elif self.mode == "dual":
            xacro_name = "dual_mevion.urdf.xacro"
        elif self.mode == "quad":
            xacro_name = "quad_mevion.urdf.xacro"
        pkg_fullpath = get_package_share_directory("mevion")
        xacro_fullpath = os.path.join(pkg_fullpath, "models", xacro_name)
        urdf_fullpath = xacro_fullpath.replace(".xacro", "")
        subprocess.call(["xacro", xacro_fullpath, "-o", urdf_fullpath])
        self.pinocchio_model = pinocchio.buildModelFromUrdf(urdf_fullpath)
        self.pinocchio_data = self.pinocchio_model.createData()
        self.joint_params = {"lower": [], "upper": [], "effort": [], "velocity": []}
        for name in self.JOINT_NAME:
            idx = self.pinocchio_model.getJointId(name)-1
            self.joint_params["lower"].append(self.pinocchio_model.lowerPositionLimit[idx])
            self.joint_params["upper"].append(self.pinocchio_model.upperPositionLimit[idx])
            self.joint_params["effort"].append(self.pinocchio_model.effortLimit[idx])
            self.joint_params["velocity"].append(self.pinocchio_model.velocityLimit[idx])
        print("joint_params: ", self.joint_params)
        self.is_pinocchio_setup = True

    def setup_spacenav(self):
        if self.is_spacenav_setup:
            return
        spacenav_thread = threading.Thread(target=self.spacenav_thread_func)
        spacenav_thread.daemon = True
        spacenav_thread.start()
        self.is_spacenav_setup = True

    def spacenav_thread_func(self, hz=100):
        import spacenav, atexit
        try:
            print("Opening connection to SpaceNav driver ...")
            spacenav.open()
            print("... connection established.")
            atexit.register(spacenav.close)
        except spacenav.ConnectionError:
            print("No connection to the SpaceNav driver. Is spacenavd running?")
            return

        with self.peripheral_state.lock:
            self.peripheral_state.spacenav_enable = True

        rate = self.create_rate(hz)
        while rclpy.ok():
            event = spacenav.poll()
            if type(event) == spacenav.MotionEvent:
                max_value = 350.0
                with self.peripheral_state.lock:
                    self.peripheral_state.spacenav[0] = event.z/max_value
                    self.peripheral_state.spacenav[1] = -event.x/max_value
                    self.peripheral_state.spacenav[2] = event.y/max_value
                    self.peripheral_state.spacenav[3] = event.rz/max_value
                    self.peripheral_state.spacenav[4] = -event.rx/max_value
                    self.peripheral_state.spacenav[5] = event.ry/max_value
            elif type(event) == spacenav.ButtonEvent:
                with self.peripheral_state.lock:
                    if event.button == 0 and event.pressed == 1:
                        self.peripheral_state.spacenav[6] = not self.peripheral_state.spacenav[6]
                    if event.button == 1 and event.pressed == 1:
                        self.peripheral_state.spacenav[7] = not self.peripheral_state.spacenav[7]
            else:
                with self.peripheral_state.lock:
                    self.peripheral_state.spacenav[0] = 0.0
                    self.peripheral_state.spacenav[1] = 0.0
                    self.peripheral_state.spacenav[2] = 0.0
                    self.peripheral_state.spacenav[3] = 0.0
                    self.peripheral_state.spacenav[4] = 0.0
                    self.peripheral_state.spacenav[5] = 0.0
            spacenav.remove_events(1)
            rate.sleep()

    def setup_mujoco(self):
        if self.is_mujoco_setup:
            return
        self.sim_thread = threading.Thread(target=self.mujoco_thread_func,
                                           daemon=True)
        self.sim_thread.start()
        self.is_mujoco_setup = True

    def mujoco_thread_func(self):
        import mujoco
        import mujoco_viewer

        if self.mode == "single":
            scene_name = "single_scene.xml"
        elif self.mode == "dual":
            scene_name = "dual_scene.xml"
        elif self.mode == "quad":
            scene_name = "quad_scene.xml"
        pkg_fullpath = get_package_share_directory("mevion")
        xml_path = os.path.join(pkg_fullpath, "models", scene_name)
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        viewer = mujoco_viewer.MujocoViewer(model, data)

        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_step(model, data)

        mujoco_joint_names = [model.joint(i).name for i in range(model.njnt)]
        print("mujoco joints: ", mujoco_joint_names)
        with self.robot_state.lock:
            for i, name in enumerate(self.JOINT_NAME):
                joint_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) # mujoco
                qpos_idx = model.jnt_qposadr[joint_idx]
                qvel_idx = model.jnt_dofadr[joint_idx]
                self.robot_state.angle[i] = data.qpos[qpos_idx]
                self.robot_state.velocity[i] = data.qvel[qvel_idx]
                self.robot_state.torque[i] = 0.0
                self.robot_state.temperature[i] = 25.0

        mujoco_actuator_names = [model.actuator(i).name for i in range(model.nu)]
        print("mujoco actuators: ", mujoco_actuator_names)
        data.ctrl[:] = 0.0

        with self.robot_state.lock:
            self.robot_state.angle = self.params["RESET_ANGLE"][:]

        with self.robot_command.lock:
            self.robot_command.angle = self.params["RESET_ANGLE"][:]

        jointstate_pub = self.create_publisher(JointState, 'joint_states', 2)
        # mevion_pub = self.create_publisher(MevionLog, 'mevion_log', 2)

        print("start mujoco thread...")
        rate = self.create_rate(P.SIM_HZ)
        viewer_count = 0

        while (viewer.is_alive and rclpy.ok() and
               not self._stop_event.is_set()):

            # viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            # viewer.cam.trackbodyid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand_link")

            with self.robot_command.lock:
                p_ref = self.robot_command.angle[:]
                v_ref = self.robot_command.velocity[:]
                kp_ref = self.robot_command.kp[:]
                kd_ref = self.robot_command.kd[:]
                tau_ref = self.robot_command.torque[:]

            mujoco_actuator_names = [model.actuator(i).name for i in range(model.nu)]
            for i, name in enumerate(self.JOINT_NAME): # mevion
                if name in mujoco_actuator_names: # mujoco
                    actuator_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) # mujoco
                    joint_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) # mujoco
                    qpos_idx = model.jnt_qposadr[joint_idx]
                    qvel_idx = model.jnt_dofadr[joint_idx]
                    error = p_ref[i] - data.qpos[qpos_idx]
                    derivative = v_ref[i] - data.qvel[qvel_idx]
                    data.ctrl[actuator_idx] = kp_ref[i]*error + kd_ref[i]*derivative + tau_ref[i]

            mujoco.mj_step(model, data)

            with self.robot_state.lock:
                for i, name in enumerate(self.JOINT_NAME):
                    if name in mujoco_actuator_names:
                        actuator_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                        joint_idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
                        qpos_idx = model.jnt_qposadr[joint_idx]
                        qvel_idx = model.jnt_dofadr[joint_idx]
                        self.robot_state.angle[i] = data.qpos[qpos_idx]
                        self.robot_state.velocity[i] = data.qvel[qvel_idx]
                        self.robot_state.torque[i] = data.ctrl[actuator_idx]
                        self.robot_state.temperature[i] = 25.0

            jointstate_msg = JointState()
            jointstate_msg.header.stamp = self.get_clock().now().to_msg()
            jointstate_msg.name = self.JOINT_NAME
            with self.robot_state.lock:
                jointstate_msg.position = self.robot_state.angle
                jointstate_msg.velocity = self.robot_state.velocity
                jointstate_msg.effort = self.robot_state.torque
            jointstate_pub.publish(jointstate_msg)

            # mevion_msg = MevionLog()
            # mevion_msg.header.stamp = self.get_clock().now().to_msg()
            # with self.robot_state.lock:
            #     mevion_msg.angle = self.robot_state.angle[:]
            #     mevion_msg.velocity = self.robot_state.velocity[:]
            #     mevion_msg.torque = self.robot_state.torque[:]
            #     mevion_msg.temperature = self.robot_state.temperature[:]
            # with self.peripheral_state.lock:
            #     mevion_msg.ref_angle = self.robot_command.angle[:]
            #     mevion_msg.ref_velocity = self.robot_command.velocity[:]
            #     mevion_msg.ref_kp = self.robot_command.kp[:]
            #     mevion_msg.ref_kd = self.robot_command.kd[:]
            #     mevion_msg.ref_torque = self.robot_command.torque[:]
            # mevion_pub.publish(mevion_msg)

            if viewer_count == 0: # render takes time...
                viewer.render()
            viewer_count = (viewer_count + 1) % 4

            rate.sleep()


    def calib_motors(self):
        if self.mode != "single":
            print("[calib_motors] this function is valid in only single mode.")
            return
        for i in range(self.N_JOINTS):
            self.motors[i] = CanMotorController(bus=self.params["DEVICE"][i], motor_id=self.params["CAN_ID"][i], motor_type=self.params["MOTOR_TYPE"][i], motor_dir=self.params["MOTOR_DIR"][i])

        print("Enabling Motors...")
        for i, motor in enumerate(self.motors):
            can_id, error_code, pos, vel, tau, tem = motor.enable_motor()
            print("Enabling Motor {} [Status] Pos: {:.3f}, Vel: {:.3f}, Tau: {:.3f}, Temp: {:.3f}".format(self.JOINT_NAME[i], pos, vel, tau, tem))
            motor.set_run_mode("CONTROL_MODE")
            with self.robot_state.lock:
                self.robot_state.angle[i] = pos
                self.robot_state.velocity[i] = vel
                self.robot_state.torque[i] = tau
                self.robot_state.temperature[i] = tem

        print("Setting Initial Offset...")
        for i, motor in enumerate(self.motors):
            offset = 0.0
            with self.robot_state.lock:
                pos_orig = self.robot_state.angle[i] * self.params["MOTOR_DIR"][i]
            if self.params["MOTOR_OFFSET_THRE"][i] and pos_orig > self.params["MOTOR_OFFSET_THRE"][i]:
                offset = -2*np.pi*self.params["MOTOR_DIR"][i]
            motor.set_angle_offset(offset)

        rate = self.create_rate(P.CAN_HZ)
        pos_list = [0]*self.N_JOINTS

        while rclpy.ok():
            for i, motor in enumerate(self.motors):
                can_id, error_code, pos, vel, tau, tem = motor.send_control_command(p_ref=0, v_ref=0, kp=0.0, kd=0.0, tau_ff=0)
                pos_list[i] = pos

            # if enter is pressed, break with non-blocking
            if select.select([sys.stdin,],[],[],0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == '\n':
                    break
            print(pos_list)
            print("Press Enter to Finish... When Calibration Pose is Reached.")

            rate.sleep()

        # motor_offset_angle = [cur-raw for (cur, raw) in zip(self.params["MOTOR_CALIB_CUR_ANGLE"], pos_list)]
        # make the above value None when self.params["MOTOR_CALIB_CUR_ANGLE"] is None
        motor_offset_angle = [None if calib_cur is None else calib_cur-raw for (calib_cur, raw) in zip(self.params["MOTOR_CALIB_CUR_ANGLE"], pos_list)]
        print("MOTOR_OFFSET_ANGLE in parameters.py should be")
        print(motor_offset_angle)

        for i, motor in enumerate(self.motors):
            can_id, error_code, pos, vel, tau, tem = motor.disable_motor()
            print("Disabling Motor {} [Status] Pos: {:.3f}, Vel: {:.3f}, Tau: {:.3f}, Temp: {:.3f}".format(self.JOINT_NAME[i], pos, vel, tau, tem))

    def setup_can(self):
        if self.is_can_setup:
            return
        self.can_bus_list = list(set(self.params["DEVICE"]))  # ["can0", "can1"]
        self.can_threads = []

        for bus_name in self.can_bus_list:
            motor_indices = [i for i, dev in enumerate(self.params["DEVICE"]) if dev == bus_name]
            t = threading.Thread(target=self.can_thread_func, args=(bus_name, motor_indices))
            t.daemon = True
            t.start()
            self.can_threads.append(t)

        self.is_can_setup = True

    def can_thread_func(self, bus_name, motor_indices):
        print(f"[{bus_name}] Setting up {len(motor_indices)} motors...")
        for i in motor_indices:
            self.motors[i] = CanMotorController(bus=self.params["DEVICE"][i], motor_id=self.params["CAN_ID"][i], motor_type=self.params["MOTOR_TYPE"][i], motor_dir=self.params["MOTOR_DIR"][i])

        time.sleep(1.0)
        print(f"[{bus_name}] Enabling Motors...")
        for i in motor_indices:
            motor = self.motors[i]
            enable_success = False
            for _ in range(10): # enable try count
                try:
                    can_id, error_code, pos, vel, tau, tem = motor.enable_motor()
                    print(f"[{bus_name}] Enabling Motor {self.JOINT_NAME[i]} [Status] Pos: {pos:.3f}, Vel: {vel:.3f}, Tau: {tau:.3f}, Temp: {tem:.3f}, CAN_ID: {self.params['CAN_ID'][i], can_id}, ERROR_CODE: {error_code}")
                except:
                    pass
                if (can_id is None) or (pos is None) or (vel is None) or (tau is None) or (tem is None):
                    print(f"[{bus_name}] try Enabling Motor again {self.JOINT_NAME[i]} (data is None)")
                    time.sleep(0.1)
                elif can_id == 0:
                    print(f"[{bus_name}] try Enabling Motor again {self.JOINT_NAME[i]} (can_id == 0)")
                    time.sleep(0.1)
                elif (tem > 0.0) and (tem < 70.0) and (abs(tau) < 1.0) and (abs(vel) < 1.0) and (abs(pos) < 10.0):
                    print(f"[{bus_name}] succeeded in Enabling Motor {self.JOINT_NAME[i]}")
                    enable_success = True
                    motor.set_run_mode("CONTROL_MODE")
                    with self.robot_state.lock:
                        self.robot_state.angle[i] = pos
                        self.robot_state.velocity[i] = vel
                        self.robot_state.torque[i] = tau
                        self.robot_state.temperature[i] = tem
                    break
                else:
                    print(f"[{bus_name}] try Enabling Motor again {self.JOINT_NAME[i]}")
                    time.sleep(0.1)
            if not enable_success:
                print(f"################ [{bus_name}] {self.JOINT_NAME[i]}'s enabling is wrong. Cannot continue the operation.")
                self.disable_all_motors()
                sys.exit()
        time.sleep(0.1)

        print(f"[{bus_name}] Setting Initial Offset...")
        for i in motor_indices:
            motor = self.motors[i]
            offset = 0.0
            with self.robot_state.lock:
                pos_orig = self.robot_state.angle[i] * self.params["MOTOR_DIR"][i]
            if self.params["MOTOR_OFFSET_THRE"][i] and pos_orig > self.params["MOTOR_OFFSET_THRE"][i]:
                offset = -2 * np.pi * self.params["MOTOR_DIR"][i]
            if self.params["MOTOR_OFFSET_ANGLE"][i]:
                offset += self.params["MOTOR_OFFSET_ANGLE"][i]
            motor.set_angle_offset(offset)
            motor.set_angle_range(self.joint_params["lower"][i], self.joint_params["upper"][i])

        for i in motor_indices:
            motor = self.motors[i]
            can_id, error_code, pos, vel, tau, tem = motor.send_control_command(p_ref=0, v_ref=0, kp=0.0, kd=0.0, tau_ff=0)
            print(f"[{bus_name}] Reset Motor {self.JOINT_NAME[i]} [Status] Pos: {pos:.3f}, Vel: {vel:.3f}, Tau: {tau:.3f}, Temp: {tem:.3f}, CAN_ID: {self.params['CAN_ID'][i], can_id}, ERROR_CODE: {error_code}")
            if (tem > 70.0) or (tem < 0.0) or (abs(tau) > 1.0) or (abs(vel) > 1.0) or (abs(pos) > 10.0):
                print(f"################ [{bus_name}] {self.JOINT_NAME[i]}'s resetting is wrong. Cannot continue the operation.")
                self.disable_all_motors()
                sys.exit()
            with self.robot_state.lock:
                self.robot_state.angle[i] = pos
                self.robot_state.velocity[i] = vel
                self.robot_state.torque[i] = tau
                self.robot_state.temperature[i] = tem

        # Calibrate CyberGear motors (hand)
        for i in motor_indices:
            motor = self.motors[i]
            if self.params["MOTOR_TYPE"][i] == "CyberGear":
                calib_pos = 0.0
                start_calib_time = time.time()
                while True:
                    can_id, error_code, calib_pos, vel, tau, tem = motor.send_control_command(p_ref=0, v_ref=0, kp=0.0, kd=0.0, tau_ff=-2.0)
                    print(f"[{bus_name}] Calib CyberGear {self.JOINT_NAME[i]} [Status] Pos: {calib_pos:.3f}, Vel: {vel:.3f}, Tau: {tau:.3f}, Temp: {tem:.3f}")
                    time.sleep(0.1)
                    if time.time() - start_calib_time > 2.0:
                        break
                motor.set_angle_offset(-calib_pos)
                pos_range = self.joint_params["upper"][i] - self.joint_params["lower"][i]
                motor.set_angle_scale(pos_range / 0.90) # 0.90 is the angle range of CyberGear
                print(f"[{bus_name}] Cyber Gear is Calibrated!")

                can_id, error_code, pos, vel, tau, tem = motor.send_control_command(p_ref=0, v_ref=0, kp=0.0, kd=0.0, tau_ff=0.0)
                with self.robot_state.lock:
                    self.robot_state.angle[i] = pos
                    self.robot_state.velocity[i] = vel
                    self.robot_state.torque[i] = tau
                    self.robot_state.temperature[i] = tem

        with self.robot_command.lock:
            self.robot_command.angle[:] = self.params["RESET_ANGLE"][:]

        print(f"[{bus_name}] Gradually Setting Motors to Initial Position...")
        start_time = time.time()
        interpolation_time = 3.0
        while True:
            t = (time.time() - start_time) / interpolation_time
            if t >= 1.0:
                break
            with self.robot_command.lock:
                p_ref = self.robot_command.angle
            for i in motor_indices:
                motor = self.motors[i]
                kp_ref = robot_utils.interpolate_linear(0.0, self.params["KP_GAIN"][i], t)
                kd_ref = robot_utils.interpolate_linear(0.0, self.params["KD_GAIN"][i], t)
                try:
                    can_id, error_code, pos, vel, tau, tem = motor.send_control_command(p_ref=p_ref[i], v_ref=0, kp=kp_ref, kd=kd_ref, tau_ff=0.0)
                except:
                    print(f"################ [{bus_name}] Can Receiver Failed for {self.JOINT_NAME[i]}")
                    can_id, error_code, pos, vel, tau, tem = motor.disable_motor()
                if (can_id is None) or (pos is None) or (vel is None) or (tau is None) or (tem is None):
                    print(f"################ [{bus_name}] Can Receiver Failed for {self.JOINT_NAME[i]}")
                    can_id, error_code, pos, vel, tau, tem = motor.disable_motor()
                with self.robot_state.lock:
                    self.robot_state.angle[i] = pos
                    self.robot_state.velocity[i] = vel
                    self.robot_state.torque[i] = tau
                    self.robot_state.temperature[i] = tem
            time.sleep(1.0 / P.CAN_HZ)

        error = 0.0
        for i in motor_indices:
            error += (self.robot_state.angle[i] - self.params["RESET_ANGLE"][i])**2
        error = np.sqrt(error)
        if error > 0.3: # [rad]
            print(f"################ [{bus_name}] The initial pose is too different from the reset pose ({error:.3f}). Cannot continue the operation.")
            self.disable_all_motors()
            sys.exit()
        else:
            print(f"[{bus_name}] Position Error from reset pose, OK: {error:.3f}")

        if bus_name == "can0":
            jointstate_pub = self.create_publisher(JointState, 'joint_states', 2)
            # mevion_pub = self.create_publisher(MevionLog, 'mevion_log', 2)

        print(f"[{bus_name}] CAN thread started")
        CAN_ID = [self.params['CAN_ID'][i] for i in motor_indices]
        CAN_ID_TO_IDX = {can_id: idx for idx, can_id in zip(motor_indices, CAN_ID)}
        rate = self.create_rate(P.CAN_HZ)
        error_count = [0] * self.N_JOINTS
        while rclpy.ok():

            with self.robot_command.lock:
                p_ref = self.robot_command.angle[:]
                v_ref = self.robot_command.velocity[:]
                kp_ref = self.robot_command.kp[:]
                kd_ref = self.robot_command.kd[:]
                tau_ref = self.robot_command.torque[:]
                # p_ref = [0.0]*self.N_JOINTS
                # v_ref = [0.0]*self.N_JOINTS
                # kp_ref[:] = [0.0]*self.N_JOINTS
                # kd_ref[:] = [0.0]*self.N_JOINTS
                # tau_ref[:] = [0.0]*self.N_JOINTS

            with self.robot_state.lock:
                pos_list = self.robot_state.angle[:]
                vel_list = self.robot_state.velocity[:]
                tau_list = self.robot_state.torque[:]
                tem_list = self.robot_state.temperature[:]

            for i in motor_indices:
                motor = self.motors[i]
                if error_count[i] > 10:
                    self.disable_all_motors()
                    sys.exit()
                success = motor.only_send_control_command(p_ref=p_ref[i], v_ref=v_ref[i], kp=kp_ref[i], kd=kd_ref[i], tau_ff=tau_ref[i])
                if not success:
                    error_count[i] += 1
                    print(f"################ [{bus_name}] Can Sender Failed for {P.JOINT_NAME[i]} ({error_count[i]})")
                    continue

            expected_ids = {self.params['CAN_ID'][i] for i in motor_indices}
            got_ids = set()
            deadline = time.monotonic() + 0.02

            while len(got_ids) < len(expected_ids):
                if time.monotonic() >= deadline:
                    print(f"################ [{bus_name}] CAN Receive Timeout")
                    break

                can_id, data, arbitration_id = self.motors[motor_indices[0]].receive_can_raw_message(timeout=10)
                if (can_id is None) or (data is None) or (arbitration_id is None):
                    print(f"################ [{bus_name}] Can Receiver Failed")
                    continue
                index = CAN_ID_TO_IDX.get(can_id)
                if index is None:
                    print(f"################ [{bus_name}] Unexpected CAN ID Received: {can_id}")
                    continue
                can_id, error_code, pos, vel, tau, tem = self.motors[index].parse_received_msg(data, arbitration_id)

                if error_code != 0:
                    print(f"################ [{bus_name}] Motor Error Code {error_code} for {self.JOINT_NAME[index]}")
                elif abs(pos-pos_list[index]) > 0.2: # 18.85*0.01 = 0.1885 [rad]
                    print(f"################ [{bus_name}] Pos Diff is Too Large; abs({pos-pos_list[index]}) {self.JOINT_NAME[index]} ({error_count[index]})")
                    error_count[index] += 1
                    continue

                pos_list[index] = pos
                vel_list[index] = vel
                tau_list[index] = tau
                tem_list[index] = tem
                got_ids.add(can_id)

                if tem > 80.0:
                    print(f"[{bus_name}] Motor {self.JOINT_NAME[index]} is Overheated!!!")
                    can_id, error_code, pos, vel, tau, tem = self.motors[index].disable_motor()
                    print(f"################ [{bus_name}] Disabling Motor {self.JOINT_NAME[index]} [Status] Pos: {pos:.3f}, Vel: {vel:.3f}, Tau: {tau:.3f}, Temp: {tem:.3f}")

            # for i in motor_indices:
            #     print(f"[{bus_name}] Motor {CAN_ID[i]} [Status] Pos: {pos_list[i]:.3f}, Vel: {vel_list[i]:.3f}, Tau: {tau_list[i]:.3f}, Temp: {tem_list[i]:.3f}")

            ave_scale = 1.0
            with self.robot_state.lock:
                for i in motor_indices:
                    self.robot_state.angle[i] = pos_list[i]
                    self.robot_state.velocity[i] = (1.0-ave_scale)*self.robot_state.velocity[i] + ave_scale*vel_list[i]
                    self.robot_state.torque[i] = (1.0-ave_scale)*self.robot_state.torque[i] + ave_scale*tau_list[i]
                    self.robot_state.temperature[i] = (1.0-ave_scale)*self.robot_state.temperature[i] + ave_scale*tem_list[i]

            if bus_name == "can0":
                jointstate_msg = JointState()
                jointstate_msg.header.stamp = self.get_clock().now().to_msg()
                jointstate_msg.name = self.JOINT_NAME
                with self.robot_state.lock:
                    jointstate_msg.position = self.robot_state.angle
                    jointstate_msg.velocity = self.robot_state.velocity
                    jointstate_msg.effort = self.robot_state.torque
                jointstate_pub.publish(jointstate_msg)

                # mevion_msg = MevionLog()
                # mevion_msg.header.stamp = self.get_clock().now().to_msg()
                # with self.robot_state.lock:
                #     mevion_msg.angle = self.robot_state.angle[:]
                #     mevion_msg.velocity = self.robot_state.velocity[:]
                #     mevion_msg.torque = self.robot_state.torque[:]
                #     mevion_msg.temperature = self.robot_state.temperature[:]
                # with self.peripheral_state.lock:
                #     mevion_msg.ref_angle = self.robot_command.angle[:]
                #     mevion_msg.ref_velocity = self.robot_command.velocity[:]
                #     mevion_msg.ref_kp = self.robot_command.kp[:]
                #     mevion_msg.ref_kd = self.robot_command.kd[:]
                #     mevion_msg.ref_torque = self.robot_command.torque[:]
                # mevion_pub.publish(mevion_msg)

            rate.sleep()


### Utility

    def send_angle_vector(self, final_angle, interpolation_time, limb="arms", interpolation='minjerk', hz=50, comp_torque=True):
        limb_index = self.LIMB_INDEX[limb]
        initial_angle = []
        initial_torque = []
        with self.robot_command.lock:
            for idx in limb_index:
                initial_angle.append(self.robot_command.angle[idx])
                if comp_torque:
                    initial_torque.append(self.robot_command.torque[idx])

        if comp_torque:
            with self.robot_state.lock:
                final_torque = self.calc_joint_torque(final_angle, limb=limb)
        remaining_time = interpolation_time

        rate = self.create_rate(hz)
        while rclpy.ok():
            elapsed_time = 1.0/hz
            remaining_time = remaining_time - elapsed_time
            if interpolation_time <= 0:
                t = 1.0
            else:
                t = min(1.0, (interpolation_time-remaining_time) / interpolation_time)

            with self.robot_command.lock:
                if interpolation == 'linear':
                    for i, idx in enumerate(limb_index):
                        self.robot_command.angle[idx] = robot_utils.interpolate_linear(
                            initial_angle[i],
                            final_angle[i],
                            t)
                        if comp_torque:
                            self.robot_command.torque[idx] = robot_utils.interpolate_linear(
                                initial_torque[i],
                                final_torque[i],
                                t)
                elif interpolation == 'minjerk':
                    for i, idx in enumerate(limb_index):
                        self.robot_command.angle[idx] = robot_utils.interpolate_minjerk(
                            initial_angle[i],
                            final_angle[i],
                            t)
                        if comp_torque:
                            self.robot_command.torque[idx] = robot_utils.interpolate_minjerk(
                                initial_torque[i],
                                final_torque[i],
                                t)
            rate.sleep()
            if remaining_time <= 0:
                break
        return True

    # m.robot_model.rarm.inverse_kinematics(m.make_coords([0.3, 0.0, 0.3], [0.0, -0.3, 0.0]), rotation_axis=True)
    def make_coords(self, xyz, rpy):
        return skrobot.coordinates.Coordinates(
            pos=xyz,
            rot=rpy)

    def calc_joint_torque(self, joint_angle, limb="arms"):
        if self.pinocchio_model is None:
            self.setup_pinocchio()
        limb_index = self.LIMB_INDEX[limb]
        hand_index = self.HAND_INDEX[limb]
        pin_names = self.pinocchio_model.names.tolist()[1:] # 0 is universe
        pin_angles = np.array([0]*len(pin_names), dtype=np.float32)
        # pin_names順でのindexからJOINT_NAME順でのindexへのマッピングを作る
        pin_to_joint = [-1] * len(pin_names)
        joint_to_pin = [-1] * len(self.JOINT_NAME)
        for i, name in enumerate(pin_names):
            if name in self.JOINT_NAME:
                idx = self.JOINT_NAME.index(name)
                pin_to_joint[i] = idx
                joint_to_pin[idx] = i
            else:
                # 右手/左手の名前変換
                try:
                    idx = self.JOINT_NAME.index(name.replace("right", "left"))
                    pin_to_joint[i] = idx
                except ValueError:
                    pass
        # pin_anglesをセット
        for i, name in enumerate(pin_names):
            if pin_to_joint[i] in limb_index:
                idx = limb_index.index(pin_to_joint[i])
                pin_angles[i] = joint_angle[idx]
        pinocchio.forwardKinematics(self.pinocchio_model, self.pinocchio_data, pin_angles)
        pinocchio.updateFramePlacements(self.pinocchio_model, self.pinocchio_data)
        fext = pinocchio.StdVec_Force()
        fext.extend([pinocchio.Force.Zero() for _ in self.pinocchio_model.joints])
        tau = pinocchio.computeStaticTorque(self.pinocchio_model, self.pinocchio_data, pin_angles, fext)
        # JOINT_NAME順で返す
        result = [0.0] * len(limb_index)
        for i, idx in enumerate(limb_index):
            pin_idx = joint_to_pin[idx]
            if pin_idx == -1:
                result[i] = 0.0
            elif idx in hand_index:
                result[i] = 0.0
            else:
                result[i] = tau[pin_idx]
        return result

    def base_grasp(self, force, kp, pos, interpolation_time, max_force=0.0, limb="arms", hz=50):
        hand_index = self.HAND_INDEX[limb]
        initial_force = []
        initial_angle = []
        initial_kp = []
        with self.robot_command.lock:
            for idx in hand_index:
                initial_force.append(self.robot_command.torque[idx])
                initial_angle.append(self.robot_command.angle[idx])
                initial_kp.append(self.robot_command.kp[idx])
        remaining_time = interpolation_time

        rate = self.create_rate(hz)
        while rclpy.ok():
            elapsed_time = 1.0/hz
            remaining_time = remaining_time - elapsed_time
            if interpolation_time <= 0:
                t = 1.0
            else:
                t = min(1.0, (interpolation_time-remaining_time) / interpolation_time)

            for i, idx in enumerate(hand_index):
                with self.robot_command.lock:
                    with self.robot_state.lock:
                        current_force = self.robot_state.torque[idx]
                    if max_force < 0.0 and current_force < max_force:
                        # print("minus", current_force, max_force)
                        continue
                    if max_force > 0.0 and current_force > max_force:
                        # print("plus", current_force, max_force)
                        continue
                    self.robot_command.torque[idx] = robot_utils.interpolate_linear(
                        initial_force[i],
                        force,
                        t)
                    self.robot_command.kp[idx] = max(0.0, robot_utils.interpolate_linear(
                        initial_kp[i],
                        kp,
                        t))
                    self.robot_command.angle[idx] = robot_utils.interpolate_linear(
                        initial_angle[i],
                        pos,
                        t)
                # print(remaining_time, self.robot_command.torque[idx], self.robot_command.kp[idx], self.robot_command.angle[idx])
            rate.sleep()
            if remaining_time <= 0:
                break
        return True

    def start_grasp_force(self, limb="arms", force=30.0, max_force=70.0, interpolation_time=2.0):
        pos = self.joint_params["lower"][self.HAND_INDEX[limb][0]]
        return self.base_grasp(-max(0.0, min(force, 50.0)), 0.0, pos, interpolation_time, limb=limb, max_force=-max(0.0, min(max_force, 70.0)))

    def stop_grasp_force(self, limb="arms", force=30.0, max_force=70.0, interpolation_time=2.0):
        pos = self.joint_params["upper"][self.HAND_INDEX[limb][0]]
        return self.base_grasp(max(0.0, min(force, 50.0)), 0.0, pos, interpolation_time, limb=limb, max_force=max(0.0, min(max_force, 70.0)))

    def start_grasp_pos(self, limb="arms", force=10.0, max_force=30.0, interpolation_time=2.0, kp=10):
        pos = self.joint_params["lower"][self.HAND_INDEX[limb][0]]
        return self.base_grasp(-max(0.0, min(force, 50.0)), kp, pos, interpolation_time, limb=limb, max_force=-max(0.0, min(max_force, 70.0)))

    def stop_grasp_pos(self, limb="arms", force=10.0, max_force=30.0, interpolation_time=2.0, kp=10):
        pos = self.joint_params["upper"][self.HAND_INDEX[limb][0]]
        return self.base_grasp(max(0.0, min(force, 50.0)), kp, pos, interpolation_time, limb=limb, max_force=max(0.0, min(max_force, 70.0)))

    def gravity_comp(self, limb="arms", kp_scale=0.01, kd_scale=0.03, torque_scale=0.8, hz=50): # TODO use limb params
        self.robot_model.reset_manip_pose()
        self.send_angle_vector(self.robot_model.arms.angle_vector(), 3.0)
        print("gravity compensation start")
        rate = self.create_rate(hz)
        while rclpy.ok():
            with self.robot_state.lock and self.robot_command.lock:
                comp_torque = self.calc_joint_torque(self.robot_state.angle)
                self.robot_command.angle = self.robot_model.reset_manip_pose()
                self.robot_command.torque = [torque_scale*torque for torque in comp_torque]
                self.robot_command.kp = [kp_scale*p for p in self.params["KP_GAIN"]]
                self.robot_command.kd = [kd_scale*d for d in self.params["KD_GAIN"]]
            if select.select([sys.stdin,],[],[],0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == '\n':
                    break
            rate.sleep()
        print("gravity compensation stop")
        with self.robot_command.lock and self.robot_state.lock:
            self.robot_command.angle = self.robot_state.angle[:]
            self.robot_command.torque = [0.0]*self.N_JOINTS
            self.robot_command.kp = self.params["KP_GAIN"][:]
            self.robot_command.kd = self.params["KD_GAIN"][:]
        self.robot_model.reset_pose()
        self.send_angle_vector(self.robot_model.arms.angle_vector(), 3.0)

    def compute_dynamics_torque(self, q, dq, ddq_des, limb="arms"):
        """
        τ = M(q) * ddq_des + b(q, dq)
        where b = C(q, dq) * dq + G(q)
        """
        if self.pinocchio_model is None:
            self.setup_pinocchio()
        limb_index = self.LIMB_INDEX[limb]
        hand_index = self.HAND_INDEX[limb]
        pin_names = self.pinocchio_model.names.tolist()[1:] # 0 is universe
        pin_q = np.zeros(len(pin_names), dtype=np.float32)
        pin_dq = np.zeros(len(pin_names), dtype=np.float32)
        pin_ddq_des = np.zeros(len(pin_names), dtype=np.float32)
        zero_idx = []
        result_idx = []
        for i, name in enumerate(pin_names):
            if name in self.JOINT_NAME:
                is_add = True
                idx = self.JOINT_NAME.index(name)
            else:
                is_add = False
                idx = self.JOINT_NAME.index(name.replace("right", "left"))
            if idx >= 0:
                if is_add:
                    result_idx.append(i)
                    if idx in hand_index:
                        zero_idx.append(i)
                pin_q[i] = q[idx]
                pin_dq[i] = dq[idx]
                pin_ddq_des[i] = ddq_des[idx]
        pinocchio.computeAllTerms(self.pinocchio_model, self.pinocchio_data, pin_q, pin_dq)
        M = self.pinocchio_data.M
        b = pinocchio.nonLinearEffects(self.pinocchio_model, self.pinocchio_data, pin_q, pin_dq)
        tau = M @ pin_ddq_des + b
        result = []
        for i in result_idx:
            if i in zero_idx:
                result.append(0.0)
            else:
                result.append(tau[i])
        return result

    def compute_smooth_hand_torque(self, vel):
        norm_vel = np.abs(vel)
        if norm_vel < 0.005: # stationary
            return 2.0 # continuous opening torque
        else: # dynamic
            if 0.0 < vel:
                return 2.0
            else:
                return -0.2

    # larm state should be set in rarm command
    def unilateral_control(self, kp_scale=0.003, kd_scale=0.02, torque_scale=0.85, hz=200):
        if self.mode != "dual" and self.mode != "quad":
            print("[unilateral_control] this function is valid in only dual and quad mode.")
            return
        l_arm_index = self.LIMB_INDEX["leader"]
        l_hand_index = self.HAND_INDEX["leader"]
        f_arm_index = self.LIMB_INDEX["follower"]
        f_hand_index = self.HAND_INDEX["follower"]
        self.robot_model.reset_manip_pose()
        self.send_angle_vector(self.robot_model.arms.angle_vector(), 3.0)
        print("unilateral control start")
        rate = self.create_rate(hz)
        while rclpy.ok():
            with self.robot_state.lock and self.robot_command.lock:
                comp_torque = self.calc_joint_torque(self.robot_state.angle)
                self.robot_command.torque = [torque_scale*torque for torque in comp_torque]
                for idx in l_arm_index:
                    if idx in l_hand_index:
                        vel = self.robot_state.velocity[idx]
                        self.robot_command.kp[idx] = 0.0
                        self.robot_command.kd[idx] = 0.0
                        self.robot_command.torque[idx] = self.compute_smooth_hand_torque(vel)
                    else:
                        self.robot_command.angle[idx] = self.robot_model.reset_manip_pose()[idx]
                        self.robot_command.kp[idx] = kp_scale*self.params["KP_GAIN"][idx]
                        self.robot_command.kd[idx] = kd_scale*self.params["KD_GAIN"][idx]
                for l_index, f_index in zip(l_arm_index, f_arm_index):
                    self.robot_command.angle[f_index] = self.robot_state.angle[l_index]
            if select.select([sys.stdin,],[],[],0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == '\n':
                    break
            rate.sleep()
        print("unilateral control stop")
        with self.robot_command.lock and self.robot_state.lock:
            self.robot_command.angle = self.robot_state.angle[:]
            self.robot_command.torque = [0.0]*self.N_JOINTS
            self.robot_command.kp = self.params["KP_GAIN"][:]
            self.robot_command.kd = self.params["KD_GAIN"][:]
        self.robot_model.reset_pose()
        self.send_angle_vector(self.robot_model.arms.angle_vector(), 3.0)

    # 2ch bilateral control
    def bilateral_control_2ch(self, kp_scale=0.003, kd_scale=0.003, kp_pos=1.0, kd_pos=1.0, torque_scale=0.8, max_torque_scale=0.25, hz=200):
        if self.mode != "dual" and self.mode != "quad":
            print("[unilateral_control] this function is valid in only dual and quad mode.")
            return
        l_arm_index = self.LIMB_INDEX["leader"]
        f_arm_index = self.LIMB_INDEX["follower"]
        self.robot_model.reset_manip_pose()
        self.send_angle_vector(self.robot_model.arms.angle_vector(), 3.0)
        print("bilateral control start")
        with self.robot_command.lock:
            for idx in (l_arm_index + f_arm_index):
                self.robot_command.angle[idx] = self.robot_model.reset_manip_pose()[idx]
                self.robot_command.kp[idx] = kp_scale*self.params["KP_GAIN"][idx]
                self.robot_command.kd[idx] = kd_scale*self.params["KD_GAIN"][idx]
        rate = self.create_rate(hz)
        while rclpy.ok():
            ddq_des = np.zeros(self.N_JOINTS)
            with self.robot_state.lock:
                for l_index, f_index in zip(l_arm_index, f_arm_index):
                    l_pos = self.robot_state.angle[l_index]
                    f_pos = self.robot_state.angle[f_index]
                    ref_pos = (l_pos + f_pos) / 2.0
                    l_vel = self.robot_state.velocity[l_index]
                    f_vel = self.robot_state.velocity[f_index]
                    ref_vel = (l_vel + f_vel) / 2.0
                    l_command = kp_pos*self.params["BILATERAL_KP_GAIN"][l_index]*(ref_pos - l_pos) + kd_pos*self.params["BILATERAL_KD_GAIN"][l_index]*(ref_vel - l_vel)
                    f_command = kp_pos*self.params["BILATERAL_KP_GAIN"][f_index]*(ref_pos - f_pos) + kd_pos*self.params["BILATERAL_KD_GAIN"][f_index]*(ref_vel - f_vel)
                    ddq_des[l_index] = l_command
                    ddq_des[f_index] = f_command
                q = np.array(self.robot_state.angle[:])
                dq = np.array(self.robot_state.velocity[:])
                comp_torque = self.calc_joint_torque(self.robot_state.angle)
            # tau = self.compute_dynamics_torque(q, dq, ddq_des)

            with self.robot_command.lock:
                for idx in (l_arm_index+f_arm_index):
                    # command = (tau[idx]-comp_torque[idx]) + torque_scale*comp_torque[idx]
                    command = ddq_des[idx] + torque_scale*comp_torque[idx]
                    max_torque = max_torque_scale * self.joint_params["effort"][idx]
                    command = max(-max_torque, min(command, max_torque))
                    self.robot_command.torque[idx] = command
            if select.select([sys.stdin,],[],[],0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == '\n':
                    break
            rate.sleep()
        print("bilateral control stop")
        with self.robot_command.lock and self.robot_state.lock:
            self.robot_command.angle = self.robot_state.angle[:]
            self.robot_command.torque = [0.0]*self.N_JOINTS
            self.robot_command.kp = self.params["KP_GAIN"][:]
            self.robot_command.kd = self.params["KD_GAIN"][:]
        self.robot_model.reset_pose()
        self.send_angle_vector(self.robot_model.arms.angle_vector(), 3.0)

    # 4ch bilateral control
    def bilateral_control_4ch(self, kp_scale=0.003, kd_scale=0.003, kp_pos=0.2, kd_pos=0.1, kp_torque=0.0, torque_scale=0.8, max_torque_scale=0.25, hz=200):
        if self.mode != "dual" and self.mode != "quad":
            print("[unilateral_control] this function is valid in only dual and quad mode.")
            return
        l_arm_index = self.LIMB_INDEX["leader"]
        f_arm_index = self.LIMB_INDEX["follower"]
        self.robot_model.reset_manip_pose()
        self.send_angle_vector(self.robot_model.arms.angle_vector(), 3.0)
        print("bilateral control start")
        rate = self.create_rate(hz)
        while rclpy.ok():
            with self.robot_state.lock and self.robot_command.lock:
                comp_torque = self.calc_joint_torque(self.robot_state.angle)
                # self.robot_command.torque = [torque_scale*torque for torque in comp_torque]
                for idx in (l_arm_index + f_arm_index):
                    self.robot_command.angle[idx] = self.robot_model.reset_manip_pose()[idx]
                    self.robot_command.kp[idx] = kp_scale*self.params["KP_GAIN"][idx]
                    self.robot_command.kd[idx] = kd_scale*self.params["KD_GAIN"][idx]
                for l_index, f_index in zip(l_arm_index, f_arm_index):
                    l_pos = self.robot_state.angle[l_index]
                    f_pos = self.robot_state.angle[f_index]
                    l_vel = self.robot_state.velocity[l_index]
                    f_vel = self.robot_state.velocity[f_index]
                    l_tau = self.robot_state.torque[l_index] - torque_scale*comp_torque[l_index]
                    f_tau = self.robot_state.torque[f_index] - torque_scale*comp_torque[f_index]
                    l_command = kp_pos*self.params["BILATERAL_KP_GAIN"][l_index]*(f_pos - l_pos) + kd_pos*self.params["BILATERAL_KD_GAIN"][l_index]*(f_vel - l_vel) - kp_torque*self.params["BILATERAL_TORQUE_GAIN"][l_index]*(f_tau + l_tau) + torque_scale*comp_torque[l_index]
                    f_command = kp_pos*self.params["BILATERAL_KP_GAIN"][f_index]*(l_pos - f_pos) + kd_pos*self.params["BILATERAL_KD_GAIN"][f_index]*(l_vel - f_vel) - kp_torque*self.params["BILATERAL_TORQUE_GAIN"][f_index]*(l_tau + f_tau) + torque_scale*comp_torque[f_index]
                    l_max_torque = max_torque_scale * self.joint_params["effort"][l_index]
                    f_max_torque = max_torque_scale * self.joint_params["effort"][f_index]
                    l_command = max(-l_max_torque, min(l_command, l_max_torque))
                    f_command = max(-f_max_torque, min(f_command, f_max_torque))
                    self.robot_command.torque[l_index] = l_command
                    self.robot_command.torque[f_index] = f_command
            if select.select([sys.stdin,],[],[],0.0)[0]:
                input_char = sys.stdin.read(1)
                if input_char == '\n':
                    break
            rate.sleep()
        print("bilateral control stop")
        with self.robot_command.lock and self.robot_state.lock:
            self.robot_command.angle = self.robot_state.angle[:]
            self.robot_command.torque = [0.0]*self.N_JOINTS
            self.robot_command.kp = self.params["KP_GAIN"][:]
            self.robot_command.kd = self.params["KD_GAIN"][:]
        self.robot_model.reset_pose()
        self.send_angle_vector(self.robot_model.arms.angle_vector(), 3.0)

    # control arm pos and posture with 6dof of spacena, with inverse kinematics
    def spacenav_control(self, limb="arm", hz=100):
        if not self.is_spacenav_setup:
            self.setup_spacenav()

        self.robot_model.reset_manip_pose()
        self.send_angle_vector(getattr(self.robot_model, limb).angle_vector(), 3.0, comp_torque=True, limb=limb)
        with self.peripheral_state.lock:
            self.peripheral_state.spacenav[6] = False
            self.peripheral_state.spacenav[7] = True
        limb_index = self.LIMB_INDEX[limb]

        print("spacenav control start")
        print("# left click to stop")
        print("# right click to initialize")

        target_coords = getattr(self.robot_model, limb).end_coords.copy_worldcoords()

        rate = self.create_rate(hz)
        while rclpy.ok():
            with self.peripheral_state.lock:
                spacenav = self.peripheral_state.spacenav[:]
            if spacenav[6]:
                break
            if spacenav[7]:
                self.robot_model.reset_manip_pose()
                self.send_angle_vector(getattr(self.robot_model, limb).angle_vector(), 3.0, comp_torque=True, limb=limb)
                target_coords = getattr(self.robot_model, limb).end_coords.copy_worldcoords()
                with self.peripheral_state.lock:
                    self.peripheral_state.spacenav[7] = False
            diff_coords = self.make_coords([0.01*spacenav[0], 0.01*spacenav[1], 0.01*spacenav[2]], [0.01*spacenav[5], 0.01*spacenav[4], 0.01*spacenav[3]])
            target_coords.transform(diff_coords)
            success = getattr(self.robot_model, limb).inverse_kinematics(target_coords, rotation_axis=True)
            if type(success) is bool and not success:
                print("IK failed")
            self.send_angle_vector(getattr(self.robot_model, limb).angle_vector(), 0.00, interpolation='linear', comp_torque=True, limb=limb)
            rate.sleep()

        print("spacenav control stop")

