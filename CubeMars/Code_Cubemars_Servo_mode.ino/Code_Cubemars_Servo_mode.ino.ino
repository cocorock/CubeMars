/*
  Cubemars AK Series Servo Mode Demo
  -----------------------------------
  This sketch demonstrates how to send a servo-mode (position-mode) command
  to a Cubemars AK series motor driver using an ESP32 with an MCP2515 CAN interface.
  
  The driver must already be in servo mode before sending commands.
  
  In servo mode the CAN message uses an extended frame with the following ID format:
    • Bits [28:8]: Control mode (for Position Mode, use 4)
    • Bits [7:0] : Motor (driver) ID
  
  For example, for motor ID 104 (decimal), the extended CAN ID becomes:
       (4 << 8) | 104   =>   (0x04 << 8) | 0x68  = 0x468.
  
  The 4-byte data payload contains the target position as a signed 32-bit integer.
  According to the manual (section 5.1.5), this position is represented as:
       pos_int = (int32_t)(position * 10000.0);
  which—depending on the motor’s configuration—covers a range from about -360000 to 360000.
  
  Make sure to install the MCP_CAN library:
    https://github.com/coryjfowler/MCP_CAN_lib
*/

#include <SPI.h>
#include "mcp_can.h"

// Define MCP2515 pins (adapt as needed)
#define MCP2515_CS   5    // Chip Select pin for the MCP2515
#define MCP2515_INT  17   // Interrupt pin from the MCP2515

// Create the MCP_CAN object (using the defined CS pin)
MCP_CAN CAN0(MCP2515_CS);

/*
  send_servo_position_command()
  
  Sends a Position-Mode command in servo mode.

  Parameters:
    motorID  - The target motor’s ID (lower 8 bits in the CAN ID).
    position - Desired target position (in units that, when multiplied by 10000, fit the motor's range).
               (Check your motor’s manual for the proper range and units.)
  
  The CAN identifier is constructed as:
    CAN_ID = (control_mode << 8) | motorID,
  where for Position Mode, control_mode is 4.
  
  The data payload is 4 bytes containing the signed 32-bit target position.
*/
void send_servo_position_command(uint8_t motorID, float position) {
  // Convert position to a 32-bit integer value.
  int32_t pos_int = (int32_t)(position * 10000.0);
  uint8_t data[4];
  data[0] = (uint8_t)(pos_int >> 24);
  data[1] = (uint8_t)(pos_int >> 16);
  data[2] = (uint8_t)(pos_int >> 8);
  data[3] = (uint8_t)(pos_int & 0xFF);

  // Build the extended CAN ID for servo mode command.
  // For Position Mode, control_mode = 4.
  uint32_t canId = (((uint32_t)4) << 8) | motorID;

  // Send the CAN message. Set 'ext' flag to 1 to indicate extended frame format.
  byte sndStat = CAN0.sendMsgBuf(canId, 1, 4, data);
  
  Serial.print("Servo Position command sent to motor ID ");
  Serial.print(motorID);
  Serial.print(" (Extended CAN ID 0x");
  Serial.print(canId, HEX);
  Serial.print("): ");
  for (int i = 0; i < 4; i++) {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println(sndStat == CAN_OK ? "OK" : "Error");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {} // Wait for the Serial monitor to open
  Serial.println("Cubemars AK Series Servo Mode Demo");

  // Initialize the MCP2515 in extended mode at 500 kbps (assuming an 8MHz oscillator)
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized in Extended Mode successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
    while (1);
  }
  // Set the MCP2515 to normal mode to allow sending and receiving
  CAN0.setMode(MCP_NORMAL);
  delay(100);
}

void loop() {
  const uint8_t motorID = 104;  // Adjust motor ID if needed
  static int posIndex = 0;
  const float positions[] = {0.0, 45.0, 90.0, 30.0};  // Desired positions in degrees

  // Send the command to move the motor to the current target position.
  send_servo_position_command(motorID, positions[posIndex]);
  Serial.print("Moving to ");
  Serial.print(positions[posIndex]);
  Serial.println(" degrees");

  // Increment the index and wrap around when reaching the end of the positions array.
  posIndex = (posIndex + 1) % (sizeof(positions) / sizeof(positions[0]));

  // Check for any received CAN messages and print them (optional).
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    uint8_t len = 0;
    uint8_t buf[8];
    unsigned long canId;
    if (CAN0.readMsgBuf(&canId, &len, buf) == CAN_OK) {
      Serial.print("Received CAN message, ID: 0x");
      Serial.print(canId, HEX);
      Serial.print(" Data: ");
      for (uint8_t i = 0; i < len; i++) {
        if (buf[i] < 16) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  delay(1000);  // Wait one second before sending the next command
}
