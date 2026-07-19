import argparse
import sys
import select
import time
import numpy as np
from mevion.xiaomimotor_lib import CanMotorController


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", '-d', type=str, default="can0", help="can interface name")
    parser.add_argument("--ids", '-i', type=int, nargs="+", default=[127], help="motor ids to control (default motor id is 127)")
    parser.add_argument("--new_id", '-n', type=int, default=None, help="motor new id to change")
    parser.add_argument("--motor_type", type=str, default="RobStride03", help="motor type")
    parser.add_argument("--task", '-t', type=str, default="sense", help="[pos, torque, sense, change, bilateral]")
    parser.add_argument("--value", '-v', type=float, default=0.0, help="value used for task")
    parser.add_argument("--kp", type=float, default=10.0, help="p gain for position control")
    parser.add_argument("--kd", type=float, default=1.0, help="d gain for position control")
    parser.add_argument("--hz", type=int, default=100, help="hz to control motor")
    parser.add_argument("--time", type=float, default=10.0, help="time to control [sec]")
    args = parser.parse_args()

    print("# using Socket {} for can communication".format(args.device))
    print("# motor ids: {}".format(args.ids))
    assert args.ids is not None, "please input motor ids"
    if args.task == "bilateral":
        assert len(args.ids) == 2, "please input two motor ids for bilateral control"

    ids = args.ids
    motors = {}
    for id in ids:
        motor_dir = 1
        motors[id] = CanMotorController(
                bus=args.device,
                motor_id=id,
                motor_dir=motor_dir,
                motor_type=args.motor_type)


    if args.task == "change":

        assert(len(motors) == 1), "please input only one motor id for motor_id change task"
        motor = motors[args.ids[0]]
        can_id, error_code, pos, vel, tau, tem = motor.change_motor_id(new_motor_id=args.new_id)
        print("Changing Motor ID {} into {} | Position: {}, Velocity: {}, Torque: {}, Temp: {}".format(args.ids[0], args.new_id, pos, vel, tau, tem))
        sys.exit()

    print("Enabling Motors..")
    for motor_id, motor in motors.items():
        can_id, error_code, pos, vel, tau, tem = motor.enable_motor()
        print("Motor {} | Status: Pos: {}, Vel: {}, Torque: {}, Temp: {}".format(motor_id, pos, vel, tau, tem))
        motor.set_run_mode("CONTROL_MODE")

    pos_vec = []
    vel_vec = []
    tau_vec = []
    tem_vec = []
    for motor_id, motor in motors.items():
        can_id, error_code, pos, vel, tau, tem = motor.send_control_command(p_ref=0, v_ref=0, kp=0.0, kd=0.0, tau_ff=0)
        pos_vec.append(pos)
        vel_vec.append(vel)
        tau_vec.append(tau)
        tem_vec.append(tem)

    start_time = time.time()

    if args.task == "pos":

        try:
            for rad in np.linspace(0.0, 2*np.pi, int(args.time*args.hz)):
                for i, (motor_id, motor) in enumerate(motors.items()):
                    prev = pos_vec[i]
                    can_id, error_code, pos, vel, tau, tem = motor.send_control_command(p_ref=prev + rad, v_ref=0, kp=args.kp, kd=args.kd, tau_ff=0)
                    print("Moving Motor {} | Position: {}, Velocity: {}, Torque: {}, Temp: {}".format(motor_id, pos, vel, tau, tem))
                end_time = time.time()
                elapsed_time = end_time - start_time
                print("Time taken: {}".format(elapsed_time))
                time.sleep(max(0.0, 1.0/args.hz-elapsed_time))
                start_time = end_time
        finally:
            print("Disabling Motors...")
            for motor_id, motor in motors.items():
                can_id, error_code, pos, vel, tau, tem = motor.disable_motor()
            print("correctly disabled!")


    elif args.task == "torque":

        try:
            while True:
                for i, (motor_id, motor) in enumerate(motors.items()):
                    can_id, error_code, pos, vel, tau, tem = motor.send_control_command(p_ref=0, v_ref=0, kp=0, kd=0, tau_ff=args.value)
                    print("Moving Motor {} | Position: {}, Velocity: {}, Torque: {}, Temp: {}".format(motor_id, pos, vel, tau, tem))
                # if enter is pressed, break with non-blocking
                if select.select([sys.stdin,],[],[],0.0)[0]:
                    input_char = sys.stdin.read(1)
                    if input_char == '\n':
                        break
                end_time = time.time()
                elapsed_time = end_time - start_time
                print("Time taken: {}".format(elapsed_time))
                time.sleep(max(0.0, 1.0/args.hz-elapsed_time))
                start_time = end_time
        finally:
            print("Disabling Motors...")
            for motor_id, motor in motors.items():
                can_id, error_code, pos, vel, tau, tem = motor.disable_motor()
            print("correctly disabled!")

    elif args.task == "sense":

        try:
            while True:
                for i, (motor_id, motor) in enumerate(motors.items()):
                    can_id, error_code, pos, vel, tau, tem = motor.send_control_command(p_ref=0, v_ref=0, kp=0.0, kd=0.0, tau_ff=0)
                    print("Sensing Motor {} | Position: {}, Velocity: {}, Torque: {}, Temp: {}".format(motor_id, pos, vel, tau, tem))
                # if enter is pressed, break with non-blocking
                if select.select([sys.stdin,],[],[],0.0)[0]:
                    input_char = sys.stdin.read(1)
                    if input_char == '\n':
                        break
                end_time = time.time()
                elapsed_time = end_time - start_time
                time.sleep(max(0.0, 1.0/args.hz-elapsed_time))
                start_time = end_time
        finally:
            print("Disabling Motors...")
            for motor_id, motor in motors.items():
                can_id, error_code, pos, vel, tau, tem = motor.disable_motor()
            print("correctly disabled!")

    elif args.task == "bilateral":

        try:
            while True:
                for i, (motor_id, motor) in enumerate(motors.items()):
                    j = (i+1)%2 # the other motor index
                    pos_error = pos_vec[j]-pos_vec[i]
                    vel_error = vel_vec[j]-vel_vec[i]
                    force_feedback = -(tau_vec[j]+tau_vec[i])

                    # position symmetric bilateral control
                    # command_tau = 3.0*pos_error+0.3*vel_error

                    # 4ch bilateral control
                    command_tau = 3.0*pos_error + 0.3*vel_error + 1.0*force_feedback

                    command_tau = max(-4.0, min(command_tau, 4.0))

                    # print(motor_id, command_tau)
                    can_id, error_code, pos, vel, tau, tem = motor.send_control_command(0, 0, 0.0, 0.0, command_tau)
                    print("Moving Motor {} | Position: {}, Velocity: {}, Torque: {}, Temp: {}".format(motor_id, pos, vel, tau, tem))
                    pos_vec[i] = pos
                    vel_vec[i] = vel
                    tau_vec[i] = tau
                    tem_vec[i] = tem
                # if enter is pressed, break with non-blocking
                if select.select([sys.stdin,],[],[],0.0)[0]:
                    input_char = sys.stdin.read(1)
                    if input_char == '\n':
                        break
                end_time = time.time()
                elapsed_time = end_time - start_time
                print("Time taken: {}".format(elapsed_time))
                time.sleep(max(0.0, 1.0/args.hz-elapsed_time))
                start_time = end_time
        finally:
            print("Disabling Motors...")
            for motor_id, motor in motors.items():
                can_id, error_code, pos, vel, tau, tem = motor.disable_motor()
            print("correctly disabled!")

    else:

        import ipdb
        ipdb.set_trace()


if __name__ == "__main__":
    main()
