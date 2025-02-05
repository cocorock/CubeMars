import serial
import struct
import time
from typing import Dict, Optional

class AKMode:
    AK_PWM = 0
    AK_CURRENT = 1
    AK_CURRENT_BRAKE = 2
    AK_VELOCITY = 3
    AK_POSITION = 4
    AK_ORIGIN = 5
    AK_POSITION_VELOCITY = 6

class AK80Motor:
    def __init__(self, port='COM3', baudrate=115200):
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.rpm = 0.0

    def can_id(self, controller_id: int, mode: int) -> int:
        return controller_id | (mode << 8)

    def pack_can_message(self, can_id: int, data: bytes, length: int = 8) -> bytes:
        message = struct.pack('>BI', 0xFF, can_id)
        message += bytes([length])
        message += data + bytes([0] * (8 - len(data)))
        return message

    def set_rpm(self, controller_id: int, rpm: float) -> None:
        can_id = self.can_id(controller_id, AKMode.AK_VELOCITY)
        data = struct.pack('>i', int(rpm))
        message = self.pack_can_message(can_id, data, 4)
        self.serial.write(message)

    def read_motor_response(self) -> Optional[Dict]:
        """
        Read motor response data from serial port.
        Returns: Dictionary with motor data or None if no data available
        """
        if self.serial.in_waiting >= 8:
            response = self.serial.read(8)
            if len(response) == 8:
                pos_int = struct.unpack('>h', response[0:2])[0]
                spd_int = struct.unpack('>h', response[2:4])[0]
                cur_int = struct.unpack('>h', response[4:6])[0]
                temp = response[6]
                error = response[7]

                return {
                    'position': pos_int * 0.1,    # degrees
                    'speed': spd_int * 10.0,      # RPM
                    'current': cur_int * 0.01,    # Amperes
                    'temperature': temp,          # Celsius
                    'error': error
                }
        return None

    def print_motor_status(self, motor_data: Dict) -> None:
        """
        Print formatted motor status information
        Args:
            motor_data: Dictionary containing motor status values
        """
        status_str = (
            f"\rMotor Status: "
            f"Position: {motor_data['position']:>6.1f}° | "
            f"Speed: {motor_data['speed']:>6.0f} RPM | "
            f"Current: {motor_data['current']:>5.2f}A | "
            f"Temp: {motor_data['temperature']:>3d}°C | "
            f"Error: {motor_data['error']:>3d}"
        )
        print(status_str, end='', flush=True)

    def run(self):
        print("Motor Control Started")
        print("Commands: 'x' - increase speed | 's' - decrease speed | 'q' - quit")
        last_print_time = time.time()

        while True:
            # Check for user input
            if self.serial.in_waiting:
                received = self.serial.read().decode('ascii')
                if received == 'x':
                    self.rpm += 1500.0
                    print(f"\nCommand sent: Increase RPM to {self.rpm}")
                    self.set_rpm(0x01, self.rpm)
                elif received == 's':
                    self.rpm -= 1500.0
                    print(f"\nCommand sent: Decrease RPM to {self.rpm}")
                    self.set_rpm(0x01, self.rpm)
                elif received == 'q':
                    print("\nExiting...")
                    break

            # Read and print motor response every 100ms
            current_time = time.time()
            if current_time - last_print_time >= 0.1:
                motor_data = self.read_motor_response()
                if motor_data:
                    self.print_motor_status(motor_data)
                last_print_time = current_time

            time.sleep(0.01)

    def close(self):
        self.serial.close()

def main():
    try:
        motor = AK80Motor()  # Adjust COM port as needed
        motor.run()
    except serial.SerialException as e:
        print(f"\nSerial port error: {e}")
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    finally:
        if 'motor' in locals():
            motor.close()

if __name__ == "__main__":
    main()