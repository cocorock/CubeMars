#include <ACAN2515.h>
#include "RMDX.h"

// Motor IDs
unsigned char motorID1 = 1; // Example motor ID

// Configure CAN bus adapter
static const byte MCP2515_CS  = 5;  // CS input of MCP2515 (adapt to your design)
static const byte MCP2515_INT = 17; // INT output of MCP2515 (adapt to your design)
ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);

// Select the quartz frequency of your MCP2515 - 8MHz or 16MHz are often used
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL; // 8 MHz quartz
// Select CAN frequency: 500kbps or 1Mbps can be used
static const uint32_t CAN_BAUDRATE = 500UL * 1000UL; // 500kbps CAN

RMDX motor(&can); // Create the motor

void setup() {
    // Use serial port for debug
    Serial.begin(115200);
    Serial.println("Initializing CAN...");

    // Configure MCP2515
    SPI.begin();
    ACAN2515Settings settings(QUARTZ_FREQUENCY, CAN_BAUDRATE);
    settings.mRequestedMode = ACAN2515Settings::NormalMode;
    const uint16_t errorCode = can.begin(settings, [] { can.isr(); });

    if (errorCode == 0) {
        Serial.println("CAN initialized successfully!");
    } else {
        Serial.print("CAN initialization failed with error code: ");
        Serial.println(errorCode);
        while (true); // Halt execution if CAN initialization fails
    }

    delay(3000); // Allow time for motor initialization

    // Command motor to move 30 degrees clockwise
    sendPositionCommand(motorID1, 30.0);

    delay(2000); // Wait 2 seconds

    // Command motor to move 30 degrees counterclockwise
    sendPositionCommand(motorID1, -30.0);
}

void loop() {
    // Add any additional logic here if needed
}

// Function to send a position command to the motor
void sendPositionCommand(uint8_t motorID, float positionDegrees) {
    // Convert position in degrees to the required int32 format
    int32_t position = (int32_t)(positionDegrees * 10000.0); // Convert degrees to int32 (e.g., 30 -> 300000)

    // Construct the CAN frame
    CANMessage message;
    message.id = (4 << 8) | motorID; // Control Mode = 4 (Position Mode), motor ID
    message.len = 4; // Data length is 4 bytes
    message.data[0] = (position >> 24) & 0xFF; // MSB
    message.data[1] = (position >> 16) & 0xFF;
    message.data[2] = (position >> 8) & 0xFF;
    message.data[3] = position & 0xFF; // LSB

    // Send the CAN frame
    const bool ok = can.tryToSend(message);
    if (ok) {
        Serial.print("Position command sent to motor ");
        Serial.print(motorID);
        Serial.print(": ");
        Serial.print(positionDegrees);
        Serial.println(" degrees");
    } else {
        Serial.println("Failed to send CAN message");
    }
}