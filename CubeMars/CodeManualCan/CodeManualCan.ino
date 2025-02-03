/*
  Manual CAN Frame Sender via Serial Monitor
  -------------------------------------------
  This sketch uses the MCP2515 (with the MCP_CAN library) to send 
  a manual CAN frame. It now prompts the user to enter 16 hexadecimal 
  characters (representing 8 bytes) in one line.

  The sketch does the following:
  
   • Prompts the user via the Serial Monitor for the CAN ID (in hex).
   • Prompts for 16 hexadecimal digits (representing 8 data bytes).
   • Asks for user confirmation before sending.
   • Sends the frame.
   • Prints an acknowledgment status and any response from the CAN device.
   • If neither an acknowledgment nor a response is detected, it resets
     the MCP2515 to flush the CAN bus.
  
  Make sure you have installed the MCP_CAN library:
    https://github.com/coryjfowler/MCP_CAN_lib
*/

#include <SPI.h>
#include "mcp_can.h"

// Define MCP2515 pins (adjust as needed)
#define MCP2515_CS   5    // Chip Select pin for MCP2515
#define MCP2515_INT  17   // Interrupt pin from MCP2515

// Soft reset command for the MCP2515
#define MCP2515_RESET 0xC0

// Create the MCP_CAN object (using the defined CS pin)
MCP_CAN CAN0(MCP2515_CS);

// Function to flush/reset the MCP2515 by issuing a soft-reset command over SPI
void resetCAN() {
  Serial.println("Flushing CAN bus by resetting MCP2515...");
  digitalWrite(MCP2515_CS, LOW);
  SPI.transfer(MCP2515_RESET);
  digitalWrite(MCP2515_CS, HIGH);
  delay(10);
  
  // Reinitialize the MCP2515 (using the same parameters as before)
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 reinitialized successfully.");
    CAN0.setMode(MCP_NORMAL);
  } else {
    Serial.println("Error reinitializing MCP2515...");
  }
}

// Helper function: wait and read a single-line string from Serial.
// This function waits until at least one line is fully received.
String readInputLine() {
  while (Serial.available() == 0) {
    ; // wait for input
  }
  return Serial.readStringUntil('\n');
}

// Helper function: read a hexadecimal number from Serial Monitor input.
long readHexValue() {
  String s = readInputLine();
  s.trim();
  return strtol(s.c_str(), NULL, 16);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to open
  }
  Serial.println("\nManual CAN Frame Sender");
  Serial.println("---------------------------");
  
  // Initialize SPI for MCP2515
  SPI.begin();
  pinMode(MCP2515_CS, OUTPUT);
  digitalWrite(MCP2515_CS, HIGH);

  // Initialize MCP2515 with 500 kbps speed (assuming an 8MHz oscillator)
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
    while (1);
  }
  
  // Set the MCP2515 to NORMAL mode so it can send and receive messages.
  CAN0.setMode(MCP_NORMAL);
  delay(100);

  Serial.println("Setup complete. Follow the prompts below to send a CAN frame.");
}

void loop() {
  uint8_t canId;
  uint8_t data[8];

  // 1. Ask for CAN ID in hexadecimal
  Serial.println("\nEnter CAN ID (in hex, e.g., 1A):");
  canId = (uint8_t) readHexValue();
  Serial.print("CAN ID entered: 0x");
  Serial.println(canId, HEX);

  // 2. Ask for 8 bytes of data as 16 hexadecimal characters on one line.
  Serial.println("Enter 8 data bytes as 16 hex digits (e.g., A1B2C3D4E5F60708):");
  String inputData = readInputLine();
  inputData.trim();
  
  // Remove any spaces from the input if necessary.
  inputData.replace(" ", "");

  // Check if we have exactly 16 characters.
  if (inputData.length() != 16) {
    Serial.println("Invalid input length. Please enter exactly 16 hex digits.");
    return;  // restart loop on error.
  }

  // Parse the input string into 8 bytes.
  bool parseError = false;
  for (int i = 0; i < 8; i++) {
    String byteString = inputData.substring(i * 2, i * 2 + 2);
    char *endPtr;
    long value = strtol(byteString.c_str(), &endPtr, 16);
    if (*endPtr != '\0') {
      Serial.print("Error parsing byte ");
      Serial.println(i + 1);
      parseError = true;
      break;
    }
    data[i] = (uint8_t) value;
  }
  
  if (parseError) {
    Serial.println("Parsing error encountered. Please try again.");
    return;
  }
  
  // Display the full message for review before sending.
  Serial.print("Frame to send --> CAN ID: 0x");
  Serial.print(canId, HEX);
  Serial.print(" Data: ");
  for (int i = 0; i < 8; i++) {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // 3. Ask for confirmation before sending the frame.
  Serial.println("Send frame? (Y/N):");
  while (Serial.available() == 0) {
    ; // wait for input
  }
  char confirm = toupper((char)Serial.read());
  Serial.println(confirm);
  
  if (confirm != 'Y') {
    Serial.println("Frame sending aborted.");
    return;  // Skip sending, restart loop.
  }
  
  // 4. Send the CAN frame.
  byte sendStatus = CAN0.sendMsgBuf(canId, 0, 8, data);
  
  // Print acknowledgment based on send status.
  bool acked = false;
  if (sendStatus == CAN_OK) {
    Serial.println("ACK: Frame sent successfully to CAN device.");
    acked = true;
  } else {
    Serial.println("ERROR: Failed to send frame.");
  }
  
  // 5. Wait briefly (500 ms) to look for any incoming response.
  bool responseReceived = false;
  unsigned long startTime = millis();
  while (millis() - startTime < 500) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      uint8_t len = 0;
      uint8_t rxBuf[8];
      unsigned long rxId;
      if (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
        Serial.print("Received response from CAN bus, ID: 0x");
        Serial.print(rxId, HEX);
        Serial.print(" Data: ");
        for (uint8_t i = 0; i < len; i++) {
          if (rxBuf[i] < 16) Serial.print("0");
          Serial.print(rxBuf[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        responseReceived = true;
      }
    }
  }
  
  // 6. If neither an acknowledgment (successful send) nor a response was received,
  // reset (flush) the MCP2515 to clear the bus.
  if (!acked && !responseReceived) {
    Serial.println("No acknowledgment or response received from CAN device.");
    resetCAN();
  }
  
  // Add a small delay before next iteration.
  delay(1000);
}