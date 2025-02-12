import can
import struct
import sys

# ------------------------------
# Configuration Parameters
# ------------------------------

# CAN interface configuration
CAN_CHANNEL = "COM10"  # Replace with your actual COM port
CAN_BITRATE = 500000  # Changed to 500 kbps - more common CAN speed
TIMEOUT = 0.5  # Added timeout for CAN operations

# Motor control constants:
RPM_STEP = 1500
CONTROLLER_ID = 0x0A

# Enumeration for the modes (as defined in the Arduino code)
AK_PWM = 0
AK_CURRENT = 1
AK_CURRENT_BRAKE = 2
AK_VELOCITY = 3  # mode for setting rpm (velocity)
AK_POSITION = 4
AK_ORIGIN = 5
AK_POSITION_VELOCITY = 6

# ------------------------------
# Helper Functions
# ------------------------------

def can_id(controller_id: int, mode: int) -> int:
    """
    Create CAN ID by OR-ing the controller_id with the mode shifted into the higher byte.
    Arduino code uses: id | (mode << 8)
    """
    return controller_id | (mode << 8)

def send_can_frame(bus: can.Bus, can_id_val: int, data: bytes):
    """
    Sends a CAN frame using the python-can library with improved error handling
    """
    # Ensure data length is correct (8 bytes for most CAN frames)
    if len(data) < 8:
        data = data.ljust(8, b'\x00')  # Pad with zeros if needed

    # Create a CAN message with explicit formatting
    message = can.Message(
        arbitration_id=can_id_val,
        data=data,
        is_extended_id=True,
        dlc=8,  # Explicit data length
        is_remote_frame=False
    )

    retry_count = 3
    while retry_count > 0:
        try:
            bus.send(message)
            print(f"Sent packet: CAN ID=0x{can_id_val:03X}, Data={data.hex()}")
            return True
        except can.CanError as e:
            print(f"Failed to send CAN message: {e}")
            retry_count -= 1
            if retry_count > 0:
                print(f"Retrying... ({retry_count} attempts left)")
    return False

def comm_can_set_rpm(bus: can.Bus, controller_id: int, rpm: float):
    """
    Prepare and send the CAN command for setting the motor RPM.
    The Arduino code converts the float rpm to an int32 and packs it in big-endian order.
    """
    # Create a CAN id with mode AK_VELOCITY.
    frame_id = can_id(controller_id, AK_VELOCITY)
    # Convert the rpm float to an int32 (no scaling is done in the Arduino code)
    rpm_int = int(rpm)
    data = struct.pack('>i', rpm_int)  # big-endian 32-bit integer
    send_can_frame(bus, frame_id, data)

def decode_motor_response(message):
    """
    Decodes the motor response CAN frame based on the protocol in Chapter 5.2 of the manual.

    :param message: A `can.Message` object containing the motor response frame.
    :return: A dictionary with decoded position, speed, current, temperature, and error code.
    """
    if len(message.data) != 8:
        raise ValueError("Invalid CAN frame length. Expected 8 bytes.")

    # Extract data bytes
    data = message.data

    # Decode position (int16, range -32000 to 32000, representing -3200° to 3200°)
    position_raw = struct.unpack(">h", data[0:2])[0]  # Big-endian int16
    position = position_raw * 0.1  # Convert to degrees

    # Decode speed (int16, range -32000 to 32000, representing -320000 to 320000 ERPM)
    speed_raw = struct.unpack(">h", data[2:4])[0]  # Big-endian int16
    speed = speed_raw * 10  # Convert to ERPM

    # Decode current (int16, range -6000 to 6000, representing -60A to 60A)
    current_raw = struct.unpack(">h", data[4:6])[0]  # Big-endian int16
    current = current_raw * 0.01  # Convert to Amperes

    # Decode motor temperature (int8, range -20 to 127, representing -20°C to 127°C)
    temperature = struct.unpack("b", data[6:7])[0]  # Signed int8

    # Decode error code (uint8, 0 = no fault, other values indicate specific faults)
    error_code = data[7]

    # Return decoded values as a dictionary
    return {
        "position": position,  # Degrees
        "speed": speed,        # ERPM
        "current": current,    # Amperes
        "temperature": temperature,  # Degrees Celsius
        "error_code": error_code  # Fault code
    }

def print_motor_response(bus):
    """
    Listens for a motor response CAN frame and decodes it.

    :param bus: A `can.Bus` object for receiving CAN messages.
    """
    print("Waiting for motor response...")
    try:
        # Receive a CAN message (timeout set to 5 seconds)
        message = bus.recv(timeout=5.0)
        if message is None:
            print("No response received within the timeout.")
            return

        # Decode the motor response
        response = decode_motor_response(message)

        # Print the decoded response
        print("Motor Response:")
        print(f"  Position: {response['position']}°")
        print(f"  Speed: {response['speed']} ERPM")
        print(f"  Current: {response['current']} A")
        print(f"  Temperature: {response['temperature']}°C")
        print(f"  Error Code: {response['error_code']}")

        # Interpret the error code if it's non-zero
        if response['error_code'] != 0:
            print("  Error Detected!")
            interpret_error_code(response['error_code'])

    except can.CanError as e:
        print(f"CAN error: {e}")

def check_can_connection(bus):
    """
    Test CAN connection by sending a test message and listening for any response
    """
    print("Testing CAN connection...")

    # Send a test message
    test_id = can_id(CONTROLLER_ID, AK_VELOCITY)
    test_data = struct.pack('>i', 0)  # Send 0 RPM command

    if send_can_frame(bus, test_id, test_data):
        print("Test message sent successfully")

        # Try to receive any response
        try:
            message = bus.recv(timeout=1.0)
            if message is not None:
                print("Received response:", message)
                return True
            else:
                print("No response received")
                return False
        except can.CanError as e:
            print(f"Error receiving response: {e}")
            return False
    else:
        print("Failed to send test message")
        return False

def interpret_error_code(error_code):
    """
    Interprets the error code based on the manual.

    :param error_code: The error code from the motor response.
    """
    error_messages = {
        0: "No fault",
        1: "Motor over-temperature fault",
        2: "Over-current fault",
        3: "Over-voltage fault",
        4: "Under-voltage fault",
        5: "Encoder fault",
        6: "MOSFET over-temperature fault",
        7: "Motor stall"
    }
    print(f"  Error Code {error_code}: {error_messages.get(error_code, 'Unknown error')}")

# ------------------------------
# Main Code
# ------------------------------

def main():
    try:
        print("serial interface")
        # Try with serial backend first
        bus = can.interface(
            bustype="serial",  # Changed from slcan to serial
            channel=CAN_CHANNEL,
            bitrate=CAN_BITRATE,
            timeout=TIMEOUT
        )
    except:
        try:
            print("slcan interface")
            # Fallback to slcan if serial fails
            bus = can.interface(
                bustype="slcan",
                channel=CAN_CHANNEL,
                bitrate=CAN_BITRATE,
                timeout=TIMEOUT
            )
        except:
            print("Failed to initialize CAN interface")
            sys.exit(1)

    rpm = 0.0
    print("\nMotor RPM Control")
    print("Type 'x' to increase RPM, 's' to decrease RPM, or 'q' to quit.")

    # if not check_can_connection(bus):
        # print("CAN communication test failed")
        # sys.exit(1)
        
    try:
        while True:
            user_input = input("Enter command: ").strip().lower()
            if not user_input:
                continue

            if user_input == 'x':
                rpm += RPM_STEP
                print(f"Increasing RPM by {RPM_STEP}. New RPM: {rpm}")
            elif user_input == 's':
                rpm -= RPM_STEP
                print(f"Decreasing RPM by {RPM_STEP}. New RPM: {rpm}")
            elif user_input == 'q':
                print("Exiting.")
                break
            else:
                print("Unrecognized input. Use 'x', 's', or 'q'.")
                continue

            # Send the updated RPM value via the CAN command
            comm_can_set_rpm(bus, CONTROLLER_ID, rpm)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Exiting.")
    finally:
        # Clean up the CAN bus
        bus.shutdown()
        print("CAN interface closed.")

if __name__ == "__main__":
    main()