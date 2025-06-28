import candle_driver
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

CONTROLLER_ID = 0x8  # 8 in decimal

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

def send_can_message(channel, id, data):
    try:
        # Convert data to bytes if it's not already
        if isinstance(data, bytearray):
            data = bytes(data)

        # Debug print
        print(f"Sending CAN message: ID={hex(id)}, Data={[hex(b)[2:] for b in data]}")

        # Send the message
        channel.write(id, data)
        return True
    except Exception as e:
        print(f"Message NOT sent: {e}")
        return False

def enter_mode(channel):
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    return send_can_message(channel, CONTROLLER_ID, data)

def exit_mode(channel):
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    return send_can_message(channel, CONTROLLER_ID, data)

def zero_position(channel):
    data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])
    return send_can_message(channel, CONTROLLER_ID, data)

def pack_cmd(channel, motor_state):
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

    return send_can_message(channel, CONTROLLER_ID, bytes(buf))

def is_command_echo(data):
    """Check if the received data is likely a command echo rather than motor status"""
    if len(data) == 8:
        # Check for MIT mode command
        if data[0] == 0xFF and data[1] == 0xFF and data[7] in [0xFC, 0xFD, 0xFE]:
            return True

        # Check if first byte is not 0x17 (motor ID in status messages)
        if data[0] != 0x17:
            # This is likely a position command echo
            return True
    return False

def unpack_reply(data, motor_state):
    if data is None or len(data) < 6:
        return False

    # Skip command echoes
    if is_command_echo(data):
        print("Note: Received command echo, not motor status")
        return False

    id_received = data[0]
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    i_int = ((data[4] & 0xF) << 8) | data[5]

    motor_state.p_out = uint_to_float(p_int, P_MIN, P_MAX, 16)
    motor_state.v_out = uint_to_float(v_int, V_MIN, V_MAX, 12)
    motor_state.t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12)
    return True

def read_motor_status(channel, motor_state, max_attempts=3):
    """Read motor status with retry and echo filtering"""
    for attempt in range(max_attempts):
        try:
            frame_type, can_id, can_data, extended, ts = channel.read(100)
            print(f"Received: ID={hex(can_id)}, Data={[hex(b)[2:] for b in can_data]}")

            if can_id == CONTROLLER_ID:
                if not is_command_echo(can_data) and unpack_reply(can_data, motor_state):
                    return True
                elif attempt < max_attempts - 1:
                    print("Trying again for valid motor status...")
                    time.sleep(0.05)  # Short delay before retry
            else:
                print(f"Received message with unexpected ID: {hex(can_id)}")

        except TimeoutError:
            print(f"No response received from motor (attempt {attempt+1}/{max_attempts})")
        except Exception as e:
            print(f"Error reading data: {e}")

    return False

def print_menu():
    print("\n=== CubeMars AK80-64 Control Menu ===")
    print("1. Enter MIT Mode")
    print("2. Exit MIT Mode")
    print("3. Send Control Command")
    print("4. Read Status")
    print("5. Zero Position")
    print("6. Run Auto Test")
    print("q. Quit")
    print("Please enter your choice (1-6 or q):")

def get_float_input(prompt, min_val, max_val):
    while True:
        try:
            value = float(input(f"Enter {prompt} ({min_val} to {max_val}): "))
            if min_val <= value <= max_val:
                return value
            print(f"Value must be between {min_val} and {max_val}")
        except ValueError:
            print("Please enter a valid number")

def run_auto_test(channel, motor_state):
    """Run an automatic test sequence with predefined values"""
    print("\n=== Running Automatic Test Sequence ===")

    # Enter MIT mode
    print("Entering MIT Mode...")
    if not enter_mode(channel):
        print("Failed to enter MIT mode")
        return
    time.sleep(0.2)  # Increased delay

    # Zero position
    print("Zeroing position...")
    if not zero_position(channel):
        print("Failed to zero position")
        return
    time.sleep(0.2)  # Increased delay

    # Test different positions with fixed parameters
    test_positions = [0.0, 1.0, -1.0, 2.0, -2.0, 1.0, -1.0, 0.0]

    for pos in test_positions:
        print(f"\nSetting position to {pos}...")
        motor_state.p_in = pos
        motor_state.v_in = 0.0
        motor_state.kp_in = 5.0
        motor_state.kd_in = 1.0
        motor_state.t_in = 0.0

        if not pack_cmd(channel, motor_state):
            print(f"Failed to send command for position {pos}")
            continue
        time.sleep(0.9)  # Wait for motor to move

        # Read status with improved function
        print("Reading motor status...")
        if read_motor_status(channel, motor_state):
            print(f"Position: {motor_state.p_out:.2f}, Velocity: {motor_state.v_out:.2f}, Torque: {motor_state.t_out:.2f}")

    # Return to zero
    print("\nReturning to zero position...")
    motor_state.p_in = 0.0
    pack_cmd(channel, motor_state)
    time.sleep(0.5)

    print("=== Automatic Test Sequence Completed ===")

def main():
    # List available candle devices
    devices = candle_driver.list_devices()

    if not len(devices):
        print('No candle devices found.')
        exit()

    print(f'Found {len(devices)} candle devices.')

    # Display all available devices
    for i, device in enumerate(devices):
        print(f"Device {i}: {device.name()} - {device.path()}")

    # Select device if multiple are available
    device_index = 0
    if len(devices) > 1:
        device_index = int(input("Select device by number: "))

    device = devices[device_index]
    print(f'\nUsing device: {device.name()}')
    print(f'Device path: {device.path()}')
    print(f'Device channels: {device.channel_count()}\n')

    motor_state = MotorState()

    try:
        # Open device
        if not device.open():
            print("Failed to open device")
            return

        # Configure channel
        ch = device.channel(0)  # Use first channel
        ch.set_bitrate(1000000)  # 1Mbps

        # Start channel in normal mode
        if not ch.start():
            print("Failed to start CAN channel")
            device.close()
            return

        print("CAN bus initialized successfully!\n")

        while True:
            print_menu()
            choice = input().strip().lower()

            if choice == 'q':
                break

            elif choice == '1':
                print("Entering MIT Mode...")
                enter_mode(ch)
                time.sleep(0.1)  # Short delay after command

            elif choice == '2':
                print("Exiting MIT Mode...")
                exit_mode(ch)
                time.sleep(0.1)  # Short delay after command

            elif choice == '3':
                print("\nEntering Control Command Parameters:")
                motor_state.p_in = get_float_input("position", P_MIN, P_MAX)
                motor_state.v_in = get_float_input("velocity", V_MIN, V_MAX)
                motor_state.kp_in = get_float_input("kp", KP_MIN, KP_MAX)
                motor_state.kd_in = get_float_input("kd", KD_MIN, KD_MAX)
                motor_state.t_in = get_float_input("torque", T_MIN, T_MAX)

                print("Sending command...")
                pack_cmd(ch, motor_state)
                time.sleep(0.1)  # Short delay after command

            elif choice == '4':
                print("Reading status...")
                if read_motor_status(ch, motor_state):
                    print("\nMotor Status:")
                    print(f"Position: {motor_state.p_out:.2f}")
                    print(f"Velocity: {motor_state.v_out:.2f}")
                    print(f"Torque: {motor_state.t_out:.2f}")
                else:
                    print("Failed to get valid motor status")

            elif choice == '5':
                print("Zeroing position...")
                zero_position(ch)
                time.sleep(0.1)  # Short delay after command

            elif choice == '6':
                run_auto_test(ch, motor_state)

            else:
                print("Invalid choice! Please select 1-6 or q")

            time.sleep(0.1)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        try:
            print("Exiting MIT mode...")
            exit_mode(ch)
            print("Stopping CAN channel...")
            ch.stop()
            print("Closing device...")
            device.close()
            print("CAN bus closed")
        except Exception as e:
            print(f"Error during cleanup: {e}")

if __name__ == "__main__":
    main()