import can
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
    return id | (mode.value << 8)

def send_can_message(bus, id, data):
    msg = can.Message(
        arbitration_id=id,
        data=data,
        is_extended_id=True
    )
    try:
        bus.send(msg)
    except can.CanError:
        print("Message NOT sent")

def comm_can_set_duty(bus, controller_id, duty):
    data = struct.pack('>i', int(duty * 10000.0))
    send_can_message(bus, can_id(controller_id, AKMode.AK_PWM), data)

def comm_can_set_current(bus, controller_id, current):
    data = struct.pack('>i', int(current * 1000.0))
    send_can_message(bus, can_id(controller_id, AKMode.AK_CURRENT), data)

def comm_can_set_cb(bus, controller_id, current):
    data = struct.pack('>i', int(current * 1000.0))
    send_can_message(bus, can_id(controller_id, AKMode.AK_CURRENT_BRAKE), data)

def comm_can_set_rpm(bus, controller_id, rpm):
    data = struct.pack('>i', int(rpm))
    send_can_message(bus, can_id(controller_id, AKMode.AK_VELOCITY), data)

def comm_can_set_pos(bus, controller_id, pos):
    data = struct.pack('>i', int(pos * 10000.0))
    send_can_message(bus, can_id(controller_id, AKMode.AK_POSITION), data)

def comm_can_set_origin(bus, controller_id, set_origin_mode):
    data = struct.pack('>i', int(set_origin_mode))
    send_can_message(bus, can_id(controller_id, AKMode.AK_ORIGIN), data)

def comm_can_set_pos_spd(bus, controller_id, pos, spd, rpa):
    data = struct.pack('>iHH', int(pos * 10000.0), spd, rpa)
    send_can_message(bus, can_id(controller_id, AKMode.AK_POSITION_VELOCITY), data)

def print_motor_data(pos, spd, cur, temp, error):
    print(f"Pos: {pos:.2f}°, Spd: {spd:.2f} RPM, I: {cur:.2f} A, Temp: {temp}°C, Error: {error}")

def motor_receive(bus):
    message = bus.recv(timeout=0.1)
    if message:
        pos_int = (message.data[0] << 8) | message.data[1]
        spd_int = (message.data[2] << 8) | message.data[3]
        cur_int = (message.data[4] << 8) | message.data[5]

        pos = float(pos_int) * 0.1
        spd = float(spd_int) * 10.0
        cur = float(cur_int) * 0.01
        temp = message.data[6]
        error = message.data[7]

        print_motor_data(pos, spd, cur, temp, error)
        return pos, spd, cur, temp, error
    return None

def print_menu():
    print("\n=== Motor Control Menu ===")
    for i, option in enumerate(MENU_OPTIONS, 1):
        print(f"{i}. {option}")
    print("Enter your choice (1-7) or 'q' to quit:")

def handle_menu_choice(bus, choice):
    controller_id = 0xA  # Default controller ID

    try:
        if choice == 1:
            value = float(input("Enter PWM duty cycle (-1.0 to 1.0): "))
            comm_can_set_duty(bus, controller_id, value)

        elif choice == 2:
            value = float(input("Enter current in Amps: "))
            comm_can_set_current(bus, controller_id, value)

        elif choice == 3:
            value = float(input("Enter brake current in Amps: "))
            comm_can_set_cb(bus, controller_id, value)

        elif choice == 4:
            value = float(input("Enter RPM: "))
            comm_can_set_rpm(bus, controller_id, value)

        elif choice == 5:
            value = float(input("Enter position in degrees: "))
            comm_can_set_pos(bus, controller_id, value)

        elif choice == 6:
            value = int(input("Enter origin mode (0/1): "))
            comm_can_set_origin(bus, controller_id, value)

        elif choice == 7:
            pos = float(input("Enter position in degrees: "))
            spd = int(input("Enter speed: "))
            rpa = int(input("Enter RPA: "))
            comm_can_set_pos_spd(bus, controller_id, pos, spd, rpa)

        else:
            print("Invalid choice!")

    except ValueError:
        print("Invalid input! Please enter a number.")

def main():
    # CAN bus configuration
    can_interface = 'slcan'
    channel = 'COM11'
    bitrate = 1000000

    try:
        bus = can.interface.Bus(channel=channel, interface=can_interface, bitrate=bitrate)
        print("CAN bus initialized successfully!")

        while True:
            print_menu()
            choice = input().strip().lower()

            if choice == 'q':
                break

            try:
                choice = int(choice)
                handle_menu_choice(bus, choice)
            except ValueError:
                print("Invalid input! Please enter a number or 'q' to quit.")

            # Read motor data
            motor_data = motor_receive(bus)
            time.sleep(0.1)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()

# Created/Modified files during execution:
# No files are created or modified during execution