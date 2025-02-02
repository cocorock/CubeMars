/*
  Cubemars AK80-64 MIT Mode Demo (Updated)
  ------------------------------------------
  This sketch demonstrates how to send MIT‐mode commands to a Cubemars
  AK80‑64 motor using an ESP32 connected to an MCP2515 CAN interface.
  
  Changes made:
    • The functions that use the motor's ID now receive the ID as a parameter.
    • In this demo the motor ID is set to 104 (decimal).
  
  Reference: Cubemars AK Series Module Driver User Manual, Chapter 5.3 (MIT Mode Communication Protocol)
  
  This demo sends:
    • A special “enter motor control mode” command (sent via the broadcast ID 0x00)
    • A motion command with desired parameters (position, speed, KP, KD, and feed‐forward torque)
  
  Make sure you have installed the MCP_CAN library:
    https://github.com/coryjfowler/MCP_CAN_lib
*/

#include <SPI.h>
#include "mcp_can.h"

// Define MCP2515 pins (adjust as needed)
#define MCP2515_CS   5    // Chip Select pin for MCP2515
#define MCP2515_INT  17   // Interrupt pin from MCP2515

// Create the MCP_CAN object (using the defined CS pin)
MCP_CAN CAN0(MCP2515_CS);

// ------------------------
// Float-to-uint conversion
// ------------------------
// Convert a float value (x) in [x_min, x_max] into an unsigned 
// integer with the specified number of bits.
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
  float span = x_max - x_min;
  if (x < x_min) x = x_min;
  else if (x > x_max) x = x_max;
  return (int)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

// ------------------------
// Pack MIT Mode Command Data
// ------------------------
// According to the manual, the 8-byte data layout is:
//
//   Byte:      Content
//   [0]        Position (high 8 bits)   [position: int16 (16 bits)]
//   [1]        Position (low 8 bits)
//   [2]        Speed (upper 8 bits of a 12-bit value)
//   [3]        Lower 4 bits of speed (<<4) | KP (upper 4 bits of a 12-bit value)
//   [4]        KP (low 8 bits)
//   [5]        KD (upper 8 bits of a 12-bit value)
//   [6]        Lower 4 bits of KD (<<4) | Torque (upper 4 bits of a 12-bit value)
//   [7]        Torque (low 8 bits)
//
// Parameter ranges for the AK80‑64 motor are assumed as follows:
//   Position (rad):       -12.5 to 12.5    (16 bits)
//   Speed (rad/s):        -50.0 to 50.0    (12 bits)
//   KP:                   0 to 500         (12 bits)
//   KD:                   0 to 5           (12 bits)
//   Feed-forward Torque (N.m): -45 to 45   (12 bits)
//
void pack_cmd(uint8_t *data, float p_des, float v_des, float kp, float kd, float t_ff) {
  // Define parameter ranges:
  const float P_MIN = -12.5f, P_MAX = 12.5f;
  const float V_MIN = -45.0f, V_MAX = 45.0f;
  const float Kp_MIN = 0.0f,   Kp_MAX = 500.0f;
  const float Kd_MIN = 0.0f,   Kd_MAX = 5.0f;
  const float T_MIN = -18.0f,  T_MAX = 18.0f;

  // Convert floats to unsigned ints using the specified bit widths:
  int p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);  // 16 bits for position
  int v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);    // 12 bits for speed
  int kp_int = float_to_uint(kp,   Kp_MIN, Kp_MAX, 12);   // 12 bits for KP
  int kd_int = float_to_uint(kd,   Kd_MIN, Kd_MAX, 12);   // 12 bits for KD
  int t_int  = float_to_uint(t_ff, T_MIN, T_MAX, 12);      // 12 bits for torque

  // Pack the data into the 8-byte frame:
  data[0] = p_int >> 8;                             // Position high 8 bits
  data[1] = p_int & 0xFF;                           // Position low 8 bits
  data[2] = v_int >> 4;                             // Speed high 8 bits (from 12-bit value)
  data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);     // Lower 4 bits of speed and upper 4 bits of KP
  data[4] = kp_int & 0xFF;                          // KP low 8 bits
  data[5] = kd_int >> 4;                            // KD high 8 bits (from 12-bit value)
  data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);     // Lower 4 bits of KD and upper 4 bits of Torque
  data[7] = t_int & 0xFF;                           // Torque low 8 bits
}

// ------------------------
// Send Special MIT Mode Command
// ------------------------
// Before using the motor, it is necessary to enter motor control mode.
// According to the manual, send the 8‑byte sequence:
//   {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}
// This command is sent as a broadcast (CAN ID 0x00).
void send_enter_motor_control_mode() {
  uint8_t cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  byte sndStat = CAN0.sendMsgBuf(0x00, 0, 8, cmd);
  Serial.print("Enter Motor Control Mode command sent: ");
  Serial.println(sndStat == CAN_OK ? "OK" : "Error");
}

// ------------------------
// Send an MIT Mode Command
// ------------------------
// This function builds the 8-byte command (using pack_cmd) and sends it using 
// a CAN ID equal to the provided motorID.
// In this demo the motor ID is passed as a parameter (e.g., 104).
void send_mit_command(uint8_t motorID, float p_des, float v_des, float kp, float kd, float t_ff) {
  uint8_t data[8];
  pack_cmd(data, p_des, v_des, kp, kd, t_ff);
  byte sndStat = CAN0.sendMsgBuf(motorID, 0, 8, data);  // Using standard frame (ext = 0)
  Serial.print("MIT Mode command sent to motor ID ");
  Serial.print(motorID);
  Serial.print(" (CAN ID 0x");
  Serial.print(motorID, HEX);
  Serial.print("): ");
  for (uint8_t i = 0; i < 8; i++) {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println(sndStat == CAN_OK ? "- OK" : "- Error");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // Wait for Serial monitor to open
  Serial.println("Cubemars AK80-64 MIT Mode Demo (Updated)");

  // Initialize MCP2515 with 500 kbps speed (assuming an 8MHz oscillator)
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
    while (1);
  }
  
  // Set the MCP2515 to normal mode to allow sending and receiving.
  CAN0.setMode(MCP_NORMAL);
  delay(100);

  // Enter Motor Control Mode (this is a broadcast command)
  send_enter_motor_control_mode();
  Serial.println("Waiting to enterd MIT mode");
  delay(8000);
}

void loop() {
  // Use a static index variable to cycle through the positions
  static int posIndex = 0;
  
  // Define the motor ID (same as before)
  const uint8_t motorID = 1;
  
  // Define an array of target positions in radians:
  // 0°, 45°, 90°, then back to 0°
  float positions[] = {
    0.0f,                   // 0 degrees
    45.0f * (PI / 180.0f),    // 45 degrees in radians (~0.7854)
    90.0f * (PI / 180.0f),    // 90 degrees in radians (~1.5708)
    30.0f * (PI / 180.0f),    // 30 degrees in radians (~1.5708)
  };

  // Send an MIT mode motion command using the current target position.
  // In this example, desired speed is 0.0 rad/s, KP=10, KD=1, and feed-forward torque=0.
  send_mit_command(motorID, positions[posIndex], 0.0f, 15.0f, 1.0f, 0.0f);
  
  // Optionally print the target position in degrees
  Serial.print("Moving to position: ");
  Serial.print(positions[posIndex] * (180.0f / PI)); // converting radians to degrees for printing
  Serial.println(" degrees");
  
  // Increment the index, wrapping back to 0 when reaching the end of the array.
  posIndex = (posIndex + 1) % 4;
  
  // Check for any incoming CAN messages (same as before) and print them out.
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
  
  // Wait 1 seconds between commands
  delay(1000);
}
