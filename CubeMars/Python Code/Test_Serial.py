import serial
import struct
import time
# Send status over CAN, HAS TO BE DISABLE (TAB Application Functions in Cubemars Software)

class AKServoMotor:
    def __init__(self, port="COM10", baudrate=921600):
        """Initialize serial connection to the AK servo motor."""
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )

    def send_position_command(self, position):
        """Send position command to the motor."""
        # Convert position to int32 (position * 10000)
        pos_int = int(position * 10000)

        # Create command data
        cmd_data = bytearray([0x09])  # Position command ID
        cmd_data.extend(struct.pack('>i', pos_int))  # Position data (4 bytes, big-endian)

        # Calculate packet length and CRC
        length = len(cmd_data)
        crc = self.crc16(cmd_data)

        # Build complete packet
        packet = bytearray([0x02])  # Frame head
        packet.append(length)  # Data length
        packet.extend(cmd_data)  # Command data
        packet.extend(struct.pack('>H', crc))  # CRC (2 bytes)
        packet.append(0x03)  # Frame tail

        # Send packet
        self.serial.write(packet)
        time.sleep(0.01)  # Small delay to ensure command is sent

    def read_motor_status(self):
        """Read motor status and return parsed values."""
        try:
            # Send status request command (COMM_GET_VALUES)
            cmd_data = bytearray([0x04])
            length = len(cmd_data)
            crc = self.crc16(cmd_data)

            packet = bytearray([0x02, length])
            packet.extend(cmd_data)
            packet.extend(struct.pack('>H', crc))
            packet.append(0x03)
            # print(f"Sent command: {packet}")
            
            self.serial.write(packet)
            time.sleep(0.01)  # Wait for response

            # Read response
            response = self._read_response()
            if response:
                return self._parse_status_response(response)
            return None
        except Exception as e:
            print(f"Error reading motor status: {e}")
            return None

    def _read_response(self):
        """Read and validate response from the motor."""
        try:
            # Wait for frame head (0x02)
            while True:
                if self.serial.in_waiting == 0:
                    time.sleep(0.001)
                    continue

                byte = self.serial.read(1)
                if byte == b'\x02':
                    # print(f"read_response: \nbyte: {hex(ord(byte))}") 
                    break

            # Read length
            length = self.serial.read(1)
            # print(f"length: {hex(ord(length))}") 
            if not length:
                return None
            data_length = length[0]

            # Read data
            data = self.serial.read(data_length)
            # print(f"data: {data.hex()}") 
            if len(data) != data_length:
                return None

            # Read CRC and frame tail
            crc_bytes = self.serial.read(2)
            tail = self.serial.read(1)

            if len(crc_bytes) != 2 or tail != b'\x03':
                return None

            return data

        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
            return None

    def _parse_status_response(self, data):
        """Parse status response data according to protocol."""
        try:
            if len(data) < 40:  # Minimum length check
                raise ValueError(f"Response data too short: {len(data)} bytes")
            # Debbuging ...
            # print(f"Raw data length: {len(data)}")
            # print(f"Raw data: {data.hex()}")
            
            index = 1
            # Debbuging ...
            # print(f"MOS temp raw: {data[index:index+2].hex()} | decimal: {struct.unpack('>h', data[index:index+2])[0] / 10.0}")
            # print(f"Motor temp raw: {data[index+2:index+4].hex()} | decimal: {struct.unpack('>h', data[index+2:index+4])[0] / 10.0}")
            # print(f"Current out raw: {data[index+4:index+8].hex()} | decimal: {struct.unpack('>i', data[index+4:index+8])[0] / 100.0}")
            # print(f"Current in raw: {data[index+8:index+12].hex()} | decimal: {struct.unpack('>i', data[index+8:index+12])[0] / 100.0}")
            # print(f"Id current raw: {data[index+12:index+16].hex()} | decimal: {struct.unpack('>i', data[index+12:index+16])[0] / 100.0}")
            # print(f"Iq current raw: {data[index+16:index+20].hex()} | decimal: {struct.unpack('>i', data[index+16:index+20])[0] / 100.0}")
            # print(f"Throttle raw: {data[index+20:index+22].hex()} | decimal: {struct.unpack('>h', data[index+20:index+22])[0] / 1000.0}")
            # print(f"Speed raw: {data[index+22:index+26].hex()} | decimal: {struct.unpack('>i', data[index+22:index+26])[0]}")
            # print(f"Voltage in raw: {data[index+26:index+28].hex()} | decimal: {struct.unpack('>h', data[index+26:index+28])[0] / 10.0}")
            # print(f"Reserved raw: {data[index+28:index+52].hex()}")
            # print(f"Motor status raw: {data[index+52:index+53].hex()} | decimal: {struct.unpack('>B', data[index+52:index+53])[0]}")
            # print(f"Position raw: {data[index+53:index+57].hex()} | decimal: {struct.unpack('>i', data[index+53:index+57])[0] / 1000000.0}")
            # print(f"Motor ID raw: {data[index+57:index+58].hex()} | decimal: {struct.unpack('>B', data[index+57:index+58])[0]}")
            # print(f"Reserved temp raw: {data[index+58:index+64].hex()}")  # Removed unpack as 6 bytes doesn't match standard sizes
            # print(f"Vd voltage raw: {data[index+64:index+68].hex()} | decimal: {struct.unpack('>i', data[index+64:index+68])[0] / 1000.0}")
            # print(f"Vq voltage raw: {data[index+68:index+72].hex()} | decimal: {struct.unpack('>i', data[index+68:index+72])[0] / 1000.0}")

            status = {
                'mos_temp': struct.unpack('>h', data[index:index+2])[0] / 10.0,
                'motor_temp': struct.unpack('>h', data[index+2:index+4])[0] / 10.0,
                'current_out': struct.unpack('>i', data[index+4:index+8])[0] / 100.0,
                'current_in': struct.unpack('>i', data[index+8:index+12])[0] / 100.0,
                'id_current': struct.unpack('>i', data[index+12:index+16])[0] / 100.0,
                'iq_current': struct.unpack('>i', data[index+16:index+20])[0] / 100.0,
                'throttle': struct.unpack('>h', data[index+20:index+22])[0] / 1000.0,
                'speed': struct.unpack('>i', data[index+22:index+26])[0],
                'voltage_in': struct.unpack('>h', data[index+26:index+28])[0] / 10.0,
                'motor_status': struct.unpack('>B', data[index+52:index+53])[0],
                'position': struct.unpack('>i', data[index+53:index+57])[0]/1000000.0,
                'motor_id': struct.unpack('>B', data[index+57:index+58])[0],
                'reserved_temp': data[index+58:index+64].hex(),  # Keep as hex string since it's 6 bytes
                'vd_voltage': struct.unpack('>i', data[index+64:index+68])[0] / 1000.0,
                'vq_voltage': struct.unpack('>i', data[index+68:index+72])[0] / 1000.0,
            }
            # print(f"status frame: {status}")
            return status
        except Exception as e:
            print(f"\tError parsing status response: {e}")
            print(f"Raw data length: {len(data)}")
            print(f"Raw data: 0x{data.hex()}")
            return None

    def crc16(self, data):
        """Calculate CRC16 for data verification."""
        crc = 0
        for byte in data:
            crc = ((crc << 8) & 0xFF00) ^ self.CRC16_TABLE[((crc >> 8) ^ byte) & 0xFF]
        return crc

    # CRC16 lookup table (from manual)
    CRC16_TABLE = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
    ]

    def close(self):
        """Close the serial connection."""
        if self.serial.is_open:
            self.serial.close()

    def send_duty_cycle_command(self, duty_cycle):
        """Send duty cycle command to the motor.
        Args:
            duty_cycle (float): Duty cycle value (o )
        """
        # Convert duty cycle to int32 (duty_cycle * 10000)
        duty_int = int(duty_cycle * 100080)

        # Create command data
        cmd_data = bytearray([0x05])  # Duty cycle command ID
        cmd_data.extend(struct.pack('>i', duty_int))  # Duty cycle data (4 bytes, big-endian)

        # Calculate packet length and CRC
        length = len(cmd_data)
        crc = self.crc16(cmd_data)

        # Build complete packet
        packet = bytearray([0x02])  # Frame head
        packet.append(length)  # Data length
        packet.extend(cmd_data)  # Command data
        packet.extend(struct.pack('>H', crc))  # CRC (2 bytes)
        packet.append(0x03)  # Frame tail

        # Send packet
        self.serial.write(packet)
        time.sleep(0.01)  # Small delay to ensure command is sent

    def send_current_command(self, current):
        """Send current command to the motor.
        Args:
            current (float): Current value in Amperes (-60A to 60A)
        """
        # Convert current to int32 (current * 1000)
        current_int = int(current * 1000)

        # Create command data
        cmd_data = bytearray([0x06])  # Current command ID
        cmd_data.extend(struct.pack('>i', current_int))

        length = len(cmd_data)
        crc = self.crc16(cmd_data)

        packet = bytearray([0x02, length])
        packet.extend(cmd_data)
        packet.extend(struct.pack('>H', crc))
        packet.append(0x03)

        self.serial.write(packet)
        time.sleep(0.01)

    def send_brake_current_command(self, brake_current):
        """Send brake current command to the motor.
        Args:
            brake_current (float): Brake current value in Amperes (0A to 60A)
        """
        # Convert brake current to int32 (brake_current * 1000)
        brake_int = int(brake_current * 1000)

        cmd_data = bytearray([0x07])  # Brake current command ID
        cmd_data.extend(struct.pack('>i', brake_int))

        length = len(cmd_data)
        crc = self.crc16(cmd_data)

        packet = bytearray([0x02, length])
        packet.extend(cmd_data)
        packet.extend(struct.pack('>H', crc))
        packet.append(0x03)

        self.serial.write(packet)
        time.sleep(0.01)

    def send_speed_command(self, speed):
        """Send speed command to the motor.
        Args:
            speed (float): Speed in electrical RPM (-32000 to 32000)
        """
        # Convert speed to int32
        speed_int = int(speed)

        cmd_data = bytearray([0x08])  # Speed command ID
        cmd_data.extend(struct.pack('>i', speed_int))

        length = len(cmd_data)
        crc = self.crc16(cmd_data)

        packet = bytearray([0x02, length])
        packet.extend(cmd_data)
        packet.extend(struct.pack('>H', crc))
        packet.append(0x03)

        self.serial.write(packet)
        time.sleep(0.01)

    def set_origin(self, mode=0):
        """Set the current position as origin.
        Args:
            mode (int): 0 for temporary origin, 1 for permanent zero point
        """
        cmd_data = bytearray([0x5F, mode])

        length = len(cmd_data)
        crc = self.crc16(cmd_data)

        packet = bytearray([0x02, length])
        packet.extend(cmd_data)
        packet.extend(struct.pack('>H', crc))
        packet.append(0x03)

        self.serial.write(packet)
        time.sleep(0.01)

    def set_position_mode(self, mode):
        """Set position control mode.
        Args:
            mode (str): 'single' for single-turn mode (0-360°) or 'multi' for multi-turn mode (±100 turns)
        """
        if mode.lower() == 'single':
            cmd_data = bytearray([0x5D])  # Single turn mode
        else:
            cmd_data = bytearray([0x5C])  # Multi turn mode

        length = len(cmd_data)
        crc = self.crc16(cmd_data)

        packet = bytearray([0x02, length])
        packet.extend(cmd_data)
        packet.extend(struct.pack('>H', crc))
        packet.append(0x03)

        self.serial.write(packet)
        time.sleep(0.01)

    def send_position_speed_command(self, position, speed, acceleration):
        """Send position-speed command to the motor.
        Args:
            position (float): Target position in degrees
            speed (float): Maximum speed in ERPM
            acceleration (float): Acceleration in ERPM/s
        """
        # Convert values to appropriate formats
        pos_int = int(position * 1000)
        spd_int = int(speed)
        acc_int = int(acceleration)

        cmd_data = bytearray([0x5B])  # Position-speed command ID
        cmd_data.extend(struct.pack('>i', pos_int))  # Position data
        cmd_data.extend(struct.pack('>i', spd_int))  # Speed data
        cmd_data.extend(struct.pack('>i', acc_int))  # Acceleration data

        length = len(cmd_data)
        crc = self.crc16(cmd_data)

        packet = bytearray([0x02, length])
        packet.extend(cmd_data)
        packet.extend(struct.pack('>H', crc))
        packet.append(0x03)

        self.serial.write(packet)
        time.sleep(0.01)

def main():
    try:
        # Create motor instance with correct COM port and baud rate
        motor = AKServoMotor("COM10", 921600)
        print("Connected to motor successfully")

        # Example usage of different control modes
        while True:
            print("\nAvailable commands:")
            print("1. Set duty cycle")
            print("2. Set current")
            print("3. Set brake current")
            print("4. Set speed")
            print("5. Set position")
            print("6. Set origin")
            print("7. Position-speed control")
            print("8. Read motor status")
            print("9. Exit")

            choice = input("Enter command number: ")

            if choice == '1':
                duty = float(input("Enter duty cycle ( to ): "))
                motor.send_duty_cycle_command(duty)
                
            elif choice == '2':
                current = float(input("Enter current (-60A to 60A): "))
                motor.send_current_command(current)

            elif choice == '3':
                brake = float(input("Enter brake current (0A to 60A): "))
                motor.send_brake_current_command(brake)

            elif choice == '4':
                speed = float(input("Enter speed (-32000 to 32000 ERPM): "))
                motor.send_speed_command(speed)

            elif choice == '5':
                mode = input("Enter mode (single/multi): ")
                motor.set_position_mode(mode)
                pos = float(input("Enter position (degrees): "))
                motor.send_position_command(pos)
                

            elif choice == '6':
                mode = int(input("Enter origin mode (0=temporary, 1=permanent): "))
                motor.set_origin(mode)

            elif choice == '7':
                pos = float(input("Enter position (degrees): "))
                speed = float(input("Enter speed (ERPM): "))
                acc = float(input("Enter acceleration (ERPM/s): "))
                motor.send_position_speed_command(pos, speed, acc)

            elif choice == '8':
                status = motor.read_motor_status()
                if status:
                    print("\nMotor Status:")
                    print(f"\tMOS Temperature: {status['mos_temp']}°C")
                    print(f"\tMotor Temperature: {status['motor_temp']}°C")
                    print(f"Current Out: {status['current_out']}A")
                    print(f"Current In: {status['current_in']}A")
                    print(f"\tId Current: {status['id_current']}A")
                    print(f"\tIq Current: {status['iq_current']}A")
                    print(f"Throttle: {status['throttle']}")
                    print(f"Speed: {status['speed']} ERPM")
                    print(f"\tInput Voltage: {status['voltage_in']}V")
                    print(f"Motor Status: {status['motor_status']}")
                    print(f"\tPosition: {status['position']}°")
                    print(f"Motor ID: {status['motor_id']}")
                    print(f"Reserved Temp: {status['reserved_temp']}")
                    print(f"\tVd Voltage: {status['vd_voltage']}V")
                    print(f"\tVq Voltage: {status['vq_voltage']}V")
            elif choice == '9':
                break

            else:
                print("Invalid command number")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.close()

if __name__ == "__main__":
    main()