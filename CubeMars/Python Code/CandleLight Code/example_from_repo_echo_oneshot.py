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

def handle_menu_choice(channel, choice):
    controller_id = 0xA  # Default controller ID

    try:
        if choice == 1:
            value = float(input("Enter PWM duty cycle (-1.0 to 1.0): "))
            comm_can_set_duty(channel, controller_id, value)

        elif choice == 2:
            value = float(input("Enter current in Amps: "))
            comm_can_set_current(channel, controller_id, value)

        elif choice == 3:
            value = float(input("Enter brake current in Amps: "))
            comm_can_set_cb(channel, controller_id, value)

        elif choice == 4:
            value = float(input("Enter RPM: "))
            comm_can_set_rpm(channel, controller_id, value)

        elif choice == 5:
            value = float(input("Enter position in degrees: "))
            comm_can_set_pos(channel, controller_id, value)

        elif choice == 6:
            value = int(input("Enter origin mode (0/1): "))
            comm_can_set_origin(channel, controller_id, value)

        elif choice == 7:
            pos = float(input("Enter position in degrees: "))
            spd = int(input("Enter speed: "))
            rpa = int(input("Enter RPA: "))
            comm_can_set_pos_spd(channel, controller_id, pos, spd, rpa)

        else:
            print("Invalid choice!")

    except ValueError:
        print("Invalid input! Please enter a number.")

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
    print(f'Using device: {device.name()}')
    print(f'Device path: {device.path()}')
    print(f'Device channels: {device.channel_count()}')

    try:
        # Open device
        if not device.open():
            print("Failed to open device")
            return

        # Configure channel
        ch = device.channel(0)  # Use first channel
        ch.set_bitrate(1000000)  # 1Mbps

        # Start channel in ONE_SHOT mode
        if not ch.start(candle_driver.CANDLE_MODE_ONE_SHOT):
            print("Failed to start CAN channel in one-shot mode")
            device.close()
            return

        print("CAN bus initialized successfully in ONE-SHOT mode!")
        print("Note: In one-shot mode, transmission will stop after an error occurs.")

        while True:
            print_menu()
            choice = input().strip().lower()

            if choice == 'q':
                break

            try:
                choice = int(choice)
                handle_menu_choice(ch, choice)

                # After sending a message in one-shot mode, we need to restart the channel
                # if an error occurred during transmission
                ch.stop()
                ch.start(candle_driver.CANDLE_MODE_ONE_SHOT)

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