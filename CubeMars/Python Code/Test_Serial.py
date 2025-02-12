import serial
import struct
import time

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
                    break

            # Read length
            length = self.serial.read(1)
            if not length:
                return None
            data_length = length[0]

            # Read data
            data = self.serial.read(data_length)
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

            index = 0
            status = {
                'mos_temp': struct.unpack('>h', data[index:index+2])[0] / 10.0,
                'motor_temp': struct.unpack('>h', data[index+2:index+4])[0] / 10.0,
                'current_out': struct.unpack('>i', data[index+4:index+8])[0] / 100.0,
                'current_in': struct.unpack('>i', data[index+8:index+12])[0] / 100.0,
                'id_current': struct.unpack('>i', data[index+12:index+16])[0] / 100.0,
                'iq_current': struct.unpack('>i', data[index+16:index+20])[0] / 100.0,
                'speed': struct.unpack('>i', data[index+24:index+28])[0],
                'position': struct.unpack('>i', data[index+36:index+40])[0] / 10000.0
            }
            return status
        except Exception as e:
            print(f"Error parsing status response: {e}")
            print(f"Raw data length: {len(data)}")
            print(f"Raw data: {data.hex()}")
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

def main():
    try:
        # Create motor instance with correct COM port and baud rate
        motor = AKServoMotor("COM10", 921600)
        print("Connected to motor successfully")

        # Example usage: Move to different positions and read status
        test_positions = [0, 9000, 18000, 9000, 0]  # Test positions in degrees

        for pos in test_positions:
            print(f"\nMoving to position: {pos}°")
            try:
                motor.send_position_command(pos)
                time.sleep(0.5)  # Wait for movement

                # Read and display motor status
                status = motor.read_motor_status()
                if status:
                    print("\nMotor Status:")
                    print(f"Position: {status['position']}°")
                    print(f"Speed: {status['speed']} ERPM")
                    print(f"Motor Temperature: {status['motor_temp']}°C")
                    print(f"Current Out: {status['current_out']}A")
                    print(f"MOS Temperature: {status['mos_temp']}°C")
                else:
                    print("Failed to read motor status")

                time.sleep(0.5)

            except Exception as e:
                print(f"Error during movement: {e}")
                break

    except Exception as e:
        print(f"Error initializing motor: {e}")
    finally:
        try:
            motor.close()
        except:
            pass

if __name__ == "__main__":
    main()