import candle_driver
import struct
import time
from enum import Enum

class AKMode(Enum):
    AK_PWM = 0
    AK_CURRENT = 1
    AK_CURRENT_BRAKE = 2
    AK_VELOCITY = 3
    AK_POSITION = 4
    AK_ORIGIN = 5
    AK_POSITION_VELOCITY = 6

# Constants
MAX_PWM = 10000.0
MAX_CURRENT = 60000.0
MAX_VELOCITY = 10000.0
MAX_POSITION = 360000.0
MAX_POSITION_VELOCITY = 3276700.0
MIN_POSITION_VELOCITY = -327680.0
MAX_ACCELERATION = 40000.0

MENU_OPTIONS = [
    "Set PWM Duty",
    "Set Current",
    "Set Current Brake",
    "Set RPM (Velocity)",
    "Set Position",
    "Set Origin",
    "Set Position with Speed"
]

def can_id(id, mode):
    return id | (mode.value << 8) | candle_driver.CANDLE_ID_EXTENDED

def send_can_message(channel, id, data):
    try:
        channel.write(id, data)
        return True
    except Exception as e:
        print(f"Message NOT sent: {e}")
        return False

def comm_can_set_duty(channel, controller_id, duty):
    data = struct.pack('>i', int(duty * 10000.0))
    send_can_message(channel, can_id(controller_id, AKMode.AK_PWM), data)

def comm_can_set_current(channel, controller_id, current):
    data = struct.pack('>i', int(current * 1000.0))
    send_can_message(channel, can_id(controller_id, AKMode.AK_CURRENT), data)

def comm_can_set_cb(channel, controller_id, current):
    data = struct.pack('>i', int(current * 1000.0))
    send_can_message(channel, can_id(controller_id, AKMode.AK_CURRENT_BRAKE), data)

def comm_can_set_rpm(channel, controller_id, rpm):
    data = struct.pack('>i', int(rpm))
    send_can_message(channel, can_id(controller_id, AKMode.AK_VELOCITY), data)

def comm_can_set_pos(channel, controller_id, pos):
    data = struct.pack('>i', int(pos * 10000.0))
    send_can_message(channel, can_id(controller_id, AKMode.AK_POSITION), data)

def comm_can_set_origin(channel, controller_id, set_origin_mode):
    data = struct.pack('>i', int(set_origin_mode))
    send_can_message(channel, can_id(controller_id, AKMode.AK_ORIGIN), data)

def comm_can_set_pos_spd(channel, controller_id, pos, spd, rpa):
    data = struct.pack('>iHH', int(pos * 10000.0), spd, rpa)
    send_can_message(channel, can_id(controller_id, AKMode.AK_POSITION_VELOCITY), data)

def print_motor_data(pos, spd, cur, temp, error):
    print(f"Pos: {pos:.2f}°, Spd: {spd:.2f} RPM, I: {cur:.2f} A, Temp: {temp}°C, Error: {error}")

def motor_receive(channel):
    try:
        frame_type, can_id, can_data, extended, ts = channel.read(100)  # 100ms timeout

        if len(can_data) >= 8:
            pos_int = (can_data[0] << 8) | can_data[1]
            spd_int = (can_data[2] << 8) | can_data[3]
            cur_int = (can_data[4] << 8) | can_data[5]

            pos = float(pos_int) * 0.1
            spd = float(spd_int) * 10.0
            cur = float(cur_int) * 0.01
            temp = can_data[6]
            error = can_data[7]

            print_motor_data(pos, spd, cur, temp, error)
            return pos, spd, cur, temp, error
        return None
    except TimeoutError:
        # No message received within timeout
        return None
    except Exception as e:
        print(f"Error receiving data: {e}")
        return None

def print_menu():
    print("\n=== Motor Control Menu ===")
    for i, option in enumerate(MENU_OPTIONS, 1):
        print(f"{i}. {option}")
    print("Enter your choice (1-7) or 'q' to quit:")

def handle_menu_choice(channel, choice, controller_id=0xA, auto_values=None):
    try:
        if choice == 1:
            if auto_values:
                value = auto_values[0]
            else:
                value = float(input("Enter PWM duty cycle (-1.0 to 1.0): "))
            print(f"Setting PWM duty to {value}")
            comm_can_set_duty(channel, controller_id, value)

        elif choice == 2:
            if auto_values:
                value = auto_values[1]
            else:
                value = float(input("Enter current in Amps: "))
            print(f"Setting current to {value} A")
            comm_can_set_current(channel, controller_id, value)

        elif choice == 3:
            if auto_values:
                value = auto_values[2]
            else:
                value = float(input("Enter brake current in Amps: "))
            print(f"Setting brake current to {value} A")
            comm_can_set_cb(channel, controller_id, value)

        elif choice == 4:
            if auto_values:
                value = auto_values[3]
            else:
                value = float(input("Enter RPM: "))
            print(f"Setting velocity to {value} RPM")
            comm_can_set_rpm(channel, controller_id, value)

        elif choice == 5:
            if auto_values:
                value = auto_values[4]
            else:
                value = float(input("Enter position in degrees: "))
            print(f"Setting position to {value} degrees")
            comm_can_set_pos(channel, controller_id, value)

        elif choice == 6:
            if auto_values:
                value = auto_values[5]
            else:
                value = int(input("Enter origin mode (0/1): "))
            print(f"Setting origin mode to {value}")
            comm_can_set_origin(channel, controller_id, value)

        elif choice == 7:
            if auto_values:
                pos = auto_values[6]
                spd = auto_values[7]
                rpa = auto_values[8]
            else:
                pos = float(input("Enter position in degrees: "))
                spd = int(input("Enter speed: "))
                rpa = int(input("Enter RPA: "))

            print(f"Setting position to {pos} degrees with speed {spd} and RPA {rpa}")

            # Continuously send position command until target is reached or timeout
            start_time = time.time()
            timeout = 10.0  # 4 seconds timeout
            target_reached = False
            current_pos = None

            print("Sending position commands until target reached or timeout...")

            while not target_reached and (time.time() - start_time < timeout):
                # Send the position command
                comm_can_set_pos_spd(channel, controller_id, pos, spd, rpa)

                # Wait for 500ms before checking position
                time.sleep(0.5)

                # Try to get current position
                motor_data = motor_receive(channel)

                if motor_data:
                    current_pos = motor_data[0]  # Position is the first element

                    # Check if we're within 1 degree of target
                    if abs(current_pos - pos) <= 1.0:
                        target_reached = True
                        print(f"Target position reached: {current_pos:.2f}° (Target: {pos:.2f}°)")
                    else:
                        print(f"Moving to target: Current: {current_pos:.2f}°, Target: {pos:.2f}°, Diff: {abs(current_pos - pos):.2f}°")

            # Report final status
            if target_reached:
                print(f"Position control successful! Final position: {current_pos:.2f}°")
            else:
                elapsed = time.time() - start_time
                print(f"Position control timed out after {elapsed:.1f} seconds.")
                if current_pos is not None:
                    print(f"Final position: {current_pos:.2f}°, Target: {pos:.2f}°, Diff: {abs(current_pos - pos):.2f}°")
                else:
                    print("Could not read current position.")

        else:
            print("Invalid choice!")

    except ValueError:
        print("Invalid input! Please enter a number.")

def run_auto_test(channel):
    """
    Run through all menu options once with predefined values
    """
    controller_id = 0xA  # Default controller ID
    
    # Predefined test values for each menu option
    auto_values = [
        0.1,        # PWM duty (option 1)
        1.0,        # Current in Amps (option 2)
        0.5,        # Brake current in Amps (option 3)
        100.0,      # RPM (option 4)
        90.0,       # Position in degrees (option 5)
        0,          # Origin mode (option 6)
        180.0,      # Position in degrees (option 7)
        500,        # Speed for option 7
        100        # RPA for option 7
    ]
    
    print("\n=== Running Automatic Test Sequence ===")
    
    # Test each menu option
    for choice in range(1, 8):
        print(f"\nTesting option {choice}: {MENU_OPTIONS[choice-1]}")
        handle_menu_choice(channel, choice, controller_id, auto_values)
        
        # Restart channel in one-shot mode after each command
        channel.stop()
        channel.start(candle_driver.CANDLE_MODE_ONE_SHOT)
        
        # Try to receive motor data
        try:
            motor_data = motor_receive(channel)
        except Exception as e:
            print(f"Error receiving data: {e}")
        
        # Wait between commands
        time.sleep(2)
    
    print("\n=== Automatic Test Sequence Completed ===")

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
    print(f'\n-Using device: {device.name()}')
    print(f'-Device path: {device.path()}')
    print(f'-Device channels: {device.channel_count()}\n')

    try:
        # Open device
        if not device.open():
            print("Failed to open device")
            return

        # Configure channel
        ch = device.channel(0)  # Use first channel
        ch.set_bitrate(1000000)  # 1Mbps

        # Start channel in normal mode
        if not ch.start(candle_driver.CANDLE_MODE_NORMAL):
            print("Failed to start CAN channel in normal mode")
            device.close()
            return

        print("CAN bus initialized successfully in NORMAL mode!")
        print("Note: In normal mode, transmission will continue even if errors occur.")
        
        # Run automatic test sequence first
        # run_auto_test(ch)

        # Then enter interactive mode
        while True:
            print_menu()
            choice = input().strip().lower()

            if choice == 'q':
                break

            try:
                choice = int(choice)
                handle_menu_choice(ch, choice)


            except ValueError:
                print("Invalid input! Please enter a number or 'q' to quit.")

            # Read motor data
            try:
                motor_data = motor_receive(ch)
            except Exception as e:
                print(f"Error receiving data: {e}")

            time.sleep(0.1)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        try:
            ch.stop()
            device.close()
            print("CAN bus closed")
        except:
            pass

if __name__ == "__main__":
    main()