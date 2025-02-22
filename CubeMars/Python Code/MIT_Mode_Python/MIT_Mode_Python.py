import can
import struct
import time
from enum import Enum

# Constants for AK80-64 motor
P_MIN = -12.5
P_MAX = 12.5
V_MIN = -8.0
V_MAX = 8.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -144.0
T_MAX = 144.0

CONTROLLER_ID = 0x17  # 23 in decimal

class MotorState:
    def __init__(self):
        self.p_in = 0.0
        self.v_in = 0.0
        self.kp_in = 0.0
        self.kd_in = 0.50
        self.t_in = 0.0

        # Measured values
        self.p_out = 0.0
        self.v_out = 0.0
        self.t_out = 0.0

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return int((x - offset) * 4095.0 / span)
    elif bits == 16:
        return int((x - offset) * 65535.0 / span)
    return 0

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    if bits == 12:
        return (float(x_int) * span / 4095.0) + offset
    elif bits == 16:
        return (float(x_int) * span / 65535.0) + offset
    return 0.0

def enter_mode(bus):
    msg = can.Message(
        arbitration_id=CONTROLLER_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
        is_extended_id=False
    )
    bus.send(msg)

def exit_mode(bus):
    msg = can.Message(
        arbitration_id=CONTROLLER_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
        is_extended_id=False
    )
    bus.send(msg)

def zero_position(bus):
    msg = can.Message(
        arbitration_id=CONTROLLER_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
        is_extended_id=False
    )
    bus.send(msg)

def pack_cmd(bus, motor_state):
    # Constrain values
    p_des = max(min(motor_state.p_in, P_MAX), P_MIN)
    v_des = max(min(motor_state.v_in, V_MAX), V_MIN)
    kp = max(min(motor_state.kp_in, KP_MAX), KP_MIN)
    kd = max(min(motor_state.kd_in, KD_MAX), KD_MIN)
    t_ff = max(min(motor_state.t_in, T_MAX), T_MIN)

    # Convert to integers
    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

    # Pack into buffer
    buf = bytearray(8)
    buf[0] = p_int >> 8
    buf[1] = p_int & 0xFF
    buf[2] = v_int >> 4
    buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
    buf[4] = kp_int & 0xFF
    buf[5] = kd_int >> 4
    buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
    buf[7] = t_int & 0xFF

    msg = can.Message(
        arbitration_id=CONTROLLER_ID,
        data=buf,
        is_extended_id=False
    )
    bus.send(msg)

def unpack_reply(msg, motor_state):
    if msg is None:
        return

    buf = msg.data
    id_received = buf[0]
    p_int = (buf[1] << 8) | buf[2]
    v_int = (buf[3] << 4) | (buf[4] >> 4)
    i_int = ((buf[4] & 0xF) << 8) | buf[5]

    motor_state.p_out = uint_to_float(p_int, P_MIN, P_MAX, 16)
    motor_state.v_out = uint_to_float(v_int, V_MIN, V_MAX, 12)
    motor_state.t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12)

def print_menu():
    print("\n=== CubeMars AK80-64 Control Menu ===")
    print("1. Enter MIT Mode")
    print("2. Exit MIT Mode")
    print("3. Send Control Command")
    print("4. Read Status")
    print("5. Zero Position")
    print("q. Quit")
    print("Please enter your choice (1-5 or q):")

def get_float_input(prompt, min_val, max_val):
    while True:
        try:
            value = float(input(f"Enter {prompt} ({min_val} to {max_val}): "))
            if min_val <= value <= max_val:
                return value
            print(f"Value must be between {min_val} and {max_val}")
        except ValueError:
            print("Please enter a valid number")

def main():
    # CAN bus configuration
    can_interface = 'slcan'
    channel = 'COM11'
    bitrate = 1000000

    motor_state = MotorState()

    try:
        bus = can.interface.Bus(channel=channel, interface=can_interface, bitrate=bitrate)
        print("CAN bus initialized successfully!")

        # Initial setup
        zero_position(bus)
        enter_mode(bus)

        while True:
            print_menu()
            choice = input().strip().lower()

            if choice == 'q':
                break

            elif choice == '1':
                print("Entering MIT Mode...")
                enter_mode(bus)

            elif choice == '2':
                print("Exiting MIT Mode...")
                exit_mode(bus)

            elif choice == '3':
                print("\nEntering Control Command Parameters:")
                motor_state.p_in = get_float_input("position", P_MIN, P_MAX)
                motor_state.v_in = get_float_input("velocity", V_MIN, V_MAX)
                motor_state.kp_in = get_float_input("kp", KP_MIN, KP_MAX)
                motor_state.kd_in = get_float_input("kd", KD_MIN, KD_MAX)
                motor_state.t_in = get_float_input("torque", T_MIN, T_MAX)

                print("Sending command...")
                pack_cmd(bus, motor_state)

            elif choice == '4':
                print("Reading status...")
                msg = bus.recv(timeout=0.1)
                if msg:
                    unpack_reply(msg, motor_state)
                    print("\nMotor Status:")
                    print(f"Position: {motor_state.p_out:.2f}")
                    print(f"Velocity: {motor_state.v_out:.2f}")
                    print(f"Torque: {motor_state.t_out:.2f}")
                else:
                    print("No response received from motor")

            elif choice == '5':
                print("Zeroing position...")
                zero_position(bus)

            else:
                print("Invalid choice! Please select 1-5 or q")

            time.sleep(0.1)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()

# Created/Modified files during execution:
# No files are created or modified during execution