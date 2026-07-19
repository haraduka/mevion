## General

JOINT_NAME = [
        "RL_shoulder_yaw", "RL_shoulder_pitch", "RL_elbow_pitch", "RL_elbow_yaw", "RL_wrist_pitch", "RL_wrist_yaw", "RL_gripper_prismatic_left",
        "RF_shoulder_yaw", "RF_shoulder_pitch", "RF_elbow_pitch", "RF_elbow_yaw", "RF_wrist_pitch", "RF_wrist_yaw", "RF_gripper_prismatic_left",
        "LL_shoulder_yaw", "LL_shoulder_pitch", "LL_elbow_pitch", "LL_elbow_yaw", "LL_wrist_pitch", "LL_wrist_yaw", "LL_gripper_prismatic_left",
        "LF_shoulder_yaw", "LF_shoulder_pitch", "LF_elbow_pitch", "LF_elbow_yaw", "LF_wrist_pitch", "LF_wrist_yaw", "LF_gripper_prismatic_left",
        ]

DEVICE = [
        "can0", "can0", "can0", "can0", "can0", "can0", "can0",
        "can1", "can1", "can1", "can1", "can1", "can1", "can1",
        "can2", "can2", "can2", "can2", "can2", "can2", "can2",
        "can3", "can3", "can3", "can3", "can3", "can3", "can3",
        ]

CAN_ID = [
        1, 2, 3, 4, 5, 6, 7,
        8, 9, 10, 11, 12, 13, 14,
        1, 2, 3, 4, 5, 6, 7,
        15, 16, 17, 18, 19, 20, 21,
        ]

MOTOR_TYPE = [
        "RobStride03", "RobStride03", "RobStride03", "RobStride02", "RobStride02", "RobStride02", "CyberGear",
        "RobStride03", "RobStride03", "RobStride03", "RobStride02", "RobStride02", "RobStride02", "CyberGear",
        "RobStride03", "RobStride03", "RobStride03", "RobStride02", "RobStride02", "RobStride02", "CyberGear",
        "RobStride03", "RobStride03", "RobStride03", "RobStride02", "RobStride02", "RobStride02", "CyberGear",
        ]

MOTOR_DIR = [
         -1, -1, -1, -1,  1, -1, -1,
         -1, -1, -1, -1,  1, -1, -1,
         -1, -1, -1, -1,  1, -1, -1,
         -1, -1, -1, -1,  1, -1, -1,
        ]

# when the motor position is larger than this value at the calibration phase, offset is calculated as -2pi
# Note:
# the first value of motor position when bringing the motor is always from 0 to 2pi (6.3)
# for example, if the value range is [5.0, 7.0], depending on the initial position when calibrating the motor, the value range can be [5.0, 7.0] or [-1.3, 0.7]
# to avoid this, when the initial value is larger than 5 or 4, make the offset -2pi, and make the value range always from [-1.3, 0.7]
MOTOR_OFFSET_THRE = [
        5.0, None, 5.0, None, 5.0, 3.0, None,
        None, None, 4.0, None, 4.0, None, None,
        None, None, 5.0, 3.0, 4.0, None, None,
        None, None, None, 3.0, None, 3.0, None,
        ]

MOTOR_CALIB_CUR_ANGLE = [ # the joint angle when calibrating the motors
        0, -0.0288, 2.5382, 0, 0.5744, 0, None,
        0, -0.0288, 2.5382, 0, 0.5744, 0, None,
        0, -0.0288, 2.5382, 0, 0.5744, 0, None,
        0, -0.0288, 2.5382, 0, 0.5744, 0, None,
        ]

MOTOR_OFFSET_ANGLE = [ # These values are calculated by the calib_motors function
        0.9294147916044704, 2.6463115654371685, 2.507040539790445, 2.7640838087432282, -1.845299866980979, -0.8806382519687386, None,
        2.125554562602753, 1.5138329685285652, 0.6604829901409595, 4.5949178153946555, -0.487706154466095, 3.22318250170774, None,
        2.9435622995502815, 4.030366850314628, 2.0868201370942954, -1.2710183507634056, 0.19022782443170105, 3.1936529209553015, None,
        4.0392247957805125, 2.9906955071994195, 3.0856477470663837, -1.430554787036339, -2.563597331257921, -1.4973319071469788, None,
        ]

KP_GAIN = [
        70.0, 100.0, 100.0, 20.0, 20.0, 5.0, 50.0,
        70.0, 100.0, 100.0, 20.0, 20.0, 5.0, 50.0,
        70.0, 100.0, 100.0, 20.0, 20.0, 5.0, 50.0,
        70.0, 100.0, 100.0, 20.0, 20.0, 5.0, 50.0,
        ]

KD_GAIN = [
        4.0, 7.0, 7.0, 1.0, 1.0, 0.4, 1.0,
        4.0, 7.0, 7.0, 1.0, 1.0, 0.4, 1.0,
        4.0, 7.0, 7.0, 1.0, 1.0, 0.4, 1.0,
        4.0, 7.0, 7.0, 1.0, 1.0, 0.4, 1.0,
        ]
# KP_GAIN = [0.0] * 28
# KD_GAIN = [0.0] * 28

BILATERAL_KP_GAIN = [
        60.0, 120.0, 80.0, 5.0, 10.0, 2.0, 4.0,
        60.0, 120.0, 80.0, 5.0, 10.0, 2.0, 4.0,
        60.0, 120.0, 80.0, 5.0, 10.0, 2.0, 4.0,
        60.0, 120.0, 80.0, 5.0, 10.0, 2.0, 4.0,
        ]

BILATERAL_KD_GAIN = [
        2.5, 2.5, 2.5, 0.3, 0.4, 0.05, 0.2,
        2.5, 2.5, 2.5, 0.3, 0.4, 0.05, 0.2,
        2.5, 2.5, 2.5, 0.3, 0.4, 0.05, 0.2,
        2.5, 2.5, 2.5, 0.3, 0.4, 0.05, 0.2,
        ]

BILATERAL_TORQUE_GAIN = [
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
        ]

RESET_ANGLE = [
        0.0, -0.91, 2.69, 0.0, 0.53, 1.57, 0.0,
        0.0, -0.91, 2.69, 0.0, 0.53, 1.57, 0.0,
        0.0, -0.91, 2.69, 0.0, 0.53, -1.57, 0.0,
        0.0, -0.91, 2.69, 0.0, 0.53, -1.57, 0.0,
        ]

RESET_MANIP_ANGLE = [
        0.0, 0.007, 2.3, 0.0, 0.0, 0.0, 0.04,
        0.0, 0.007, 2.3, 0.0, 0.0, 0.0, 0.04,
        0.0, 0.007, 2.3, 0.0, 0.0, 0.0, 0.04,
        0.0, 0.007, 2.3, 0.0, 0.0, 0.0, 0.04,
        ]

CAN_HZ = 200

SIM_HZ = 200


## Special

JOINT_NAME_SINGLE = [
        "RL_shoulder_yaw", "RL_shoulder_pitch", "RL_elbow_pitch", "RL_elbow_yaw", "RL_wrist_pitch", "RL_wrist_yaw", "RL_gripper_prismatic_left",
]

LIMB_INDEX_SINGLE = {
        "arm": [0, 1, 2, 3, 4, 5, 6],
}

HAND_INDEX_SINGLE = {
        "arm": [6],
}

JOINT_NAME_DUAL = [
        "RL_shoulder_yaw", "RL_shoulder_pitch", "RL_elbow_pitch", "RL_elbow_yaw", "RL_wrist_pitch", "RL_wrist_yaw", "RL_gripper_prismatic_left",
        "RF_shoulder_yaw", "RF_shoulder_pitch", "RF_elbow_pitch", "RF_elbow_yaw", "RF_wrist_pitch", "RF_wrist_yaw", "RF_gripper_prismatic_left",
]

LIMB_INDEX_DUAL = {
        "leader": [0, 1, 2, 3, 4, 5, 6],
        "follower": [7, 8, 9, 10, 11, 12, 13],
        "arms": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
}

HAND_INDEX_DUAL = {
        "leader": [6],
        "follower": [13],
        "arms": [6, 13],
}

JOINT_NAME_QUAD = [
        "RL_shoulder_yaw", "RL_shoulder_pitch", "RL_elbow_pitch", "RL_elbow_yaw", "RL_wrist_pitch", "RL_wrist_yaw", "RL_gripper_prismatic_left",
        "RF_shoulder_yaw", "RF_shoulder_pitch", "RF_elbow_pitch", "RF_elbow_yaw", "RF_wrist_pitch", "RF_wrist_yaw", "RF_gripper_prismatic_left",
        "LL_shoulder_yaw", "LL_shoulder_pitch", "LL_elbow_pitch", "LL_elbow_yaw", "LL_wrist_pitch", "LL_wrist_yaw", "LL_gripper_prismatic_left",
        "LF_shoulder_yaw", "LF_shoulder_pitch", "LF_elbow_pitch", "LF_elbow_yaw", "LF_wrist_pitch", "LF_wrist_yaw", "LF_gripper_prismatic_left",
]

LIMB_INDEX_QUAD = {
        "leader": [0, 1, 2, 3, 4, 5, 6, 14, 15, 16, 17, 18, 19, 20],
        "follower": [7, 8, 9, 10, 11, 12, 13, 21, 22, 23, 24, 25, 26, 27],
        "rarm": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
        "larm": [14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27],
        "arms": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27],
}

HAND_INDEX_QUAD = {
        "leader": [6, 20],
        "follower": [13, 27],
        "rarm": [6, 13],
        "larm": [20, 27],
        "arms": [6, 13, 20, 27],
}


def _get_subset(joint_names_subset, all_joint_names, **arrays):
    indices = [all_joint_names.index(jn) for jn in joint_names_subset]
    return {name: [arr[i] for i in indices] for name, arr in arrays.items()}

SINGLE_PARAMS = _get_subset(
    JOINT_NAME_SINGLE,
    JOINT_NAME,
    DEVICE=DEVICE,
    CAN_ID=CAN_ID,
    MOTOR_TYPE=MOTOR_TYPE,
    MOTOR_DIR=MOTOR_DIR,
    MOTOR_OFFSET_THRE=MOTOR_OFFSET_THRE,
    MOTOR_CALIB_CUR_ANGLE=MOTOR_CALIB_CUR_ANGLE,
    MOTOR_OFFSET_ANGLE=MOTOR_OFFSET_ANGLE,
    KP_GAIN=KP_GAIN,
    KD_GAIN=KD_GAIN,
    BILATERAL_KP_GAIN=BILATERAL_KP_GAIN,
    BILATERAL_KD_GAIN=BILATERAL_KD_GAIN,
    BILATERAL_TORQUE_GAIN=BILATERAL_TORQUE_GAIN,
    RESET_ANGLE=RESET_ANGLE,
    RESET_MANIP_ANGLE=RESET_MANIP_ANGLE,
)

DUAL_PARAMS = _get_subset(
    JOINT_NAME_DUAL,
    JOINT_NAME,
    DEVICE=DEVICE,
    CAN_ID=CAN_ID,
    MOTOR_TYPE=MOTOR_TYPE,
    MOTOR_DIR=MOTOR_DIR,
    MOTOR_OFFSET_THRE=MOTOR_OFFSET_THRE,
    MOTOR_CALIB_CUR_ANGLE=MOTOR_CALIB_CUR_ANGLE,
    MOTOR_OFFSET_ANGLE=MOTOR_OFFSET_ANGLE,
    KP_GAIN=KP_GAIN,
    KD_GAIN=KD_GAIN,
    BILATERAL_KP_GAIN=BILATERAL_KP_GAIN,
    BILATERAL_KD_GAIN=BILATERAL_KD_GAIN,
    BILATERAL_TORQUE_GAIN=BILATERAL_TORQUE_GAIN,
    RESET_ANGLE=RESET_ANGLE,
    RESET_MANIP_ANGLE=RESET_MANIP_ANGLE,
)

QUAD_PARAMS = _get_subset(
    JOINT_NAME_QUAD,
    JOINT_NAME,
    DEVICE=DEVICE,
    CAN_ID=CAN_ID,
    MOTOR_TYPE=MOTOR_TYPE,
    MOTOR_DIR=MOTOR_DIR,
    MOTOR_OFFSET_THRE=MOTOR_OFFSET_THRE,
    MOTOR_CALIB_CUR_ANGLE=MOTOR_CALIB_CUR_ANGLE,
    MOTOR_OFFSET_ANGLE=MOTOR_OFFSET_ANGLE,
    KP_GAIN=KP_GAIN,
    KD_GAIN=KD_GAIN,
    BILATERAL_KP_GAIN=BILATERAL_KP_GAIN,
    BILATERAL_KD_GAIN=BILATERAL_KD_GAIN,
    BILATERAL_TORQUE_GAIN=BILATERAL_TORQUE_GAIN,
    RESET_ANGLE=RESET_ANGLE,
    RESET_MANIP_ANGLE=RESET_MANIP_ANGLE,
)

