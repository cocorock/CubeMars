/*
Serial TERMINAL USAGE INSTRUCTIONS:

0.One IMPORTANT thing is, when you decide MIT or Servo mode, you
  must change mode ,firstly, via R-Link, otherwise drive card can be
  burned. Another IMPORTANT thing is, the first time you use Servo
  mode, you should calibrate on application function via R-link, if you
  don’t, can bus cannot work.

1. Connect to the Arduino through serial terminal at 115200 baud rate.

2. Available Commands:
   - 'u': Increase velocity by 3.0 units
   - 'd': Decrease velocity by 3.0 units
   - 's': Enter MIT mode (must be done before controlling the motor)
   - 'e': Exit MIT mode

3. Serial Output Format:
   The program continuously prints:
   [position_input] [position_output] [velocity_output] [torque_output]

4. Initial Setup:
   - When starting, the program automatically:
     * Zeros the motor position
     * Enters MIT mode
   - Wait for "CAN BUS Shield init ok!" message before sending commands

5. Safety Limits:
   - Position is constrained between -12.5 and 12.5 rad
   - Velocity is constrained between -8.0 and 8.0 rad/s
   - Torque is constrained between -144.0 and 144.0 N.m
   - KP is constrained between 0.0 and 500.0
   - KD is constrained between 0.0 and 5.0

6. Error Handling:
   - If "CAN BUS Shield init fail" appears, check connections and restart
   - Monitor the serial output for any error messages

Note: This program uses the MIT mode protocol for motor control through CAN bus
communication. Make sure to enter MIT mode ('s' command) before attempting to 
control the motor.
*/

#include <SPI.h>
#include <mcp_can.h>

#define CONTROLLER_ID 23 //0X17
#define MCP2515_CS   5    // Chip Select pin for MCP2515
#define MCP2515_INT  17   // Interrupt pin from MCP2515


//Value Limits for AK80-64 motor
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -8.0f  // Updated for AK80-64
#define V_MAX 8.0f   // Updated for AK80-64
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -144.0f   // Updated for AK80-64
#define T_MAX 144.0f    // Updated for AK80-64

// Set Values
float p_in = 0.0f;
float v_in = 0.0f;
float kp_in = 0.0f;
float kd_in = 0.50f;
float t_in = 0.0f;

//measured values - responses from the motor
float p_out = 0.0f;  // actual position
float v_out = 0.0f;  // actual velocity
float t_out = 0.0f;  // actual torque


MCP_CAN CAN(MCP2515_CS);  // Set CS pin

void setup() {
    Serial.begin(115200);
    while (!Serial) {}  // Wait for Serial monitor to open
    Serial.println("Cubemars AK80-64 MIT Mode Demo (Updated)");

    // Initialize MCP2515 with 500 kbps speed (assuming an 8MHz oscillator)
    if (CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
      Serial.println("MCP2515 Initialized Successfully!");
    } else {
      Serial.println("Error Initializing MCP2515...");
      while (1);
    }
    pinMode(MCP2515_INT, INPUT);  // Setting interrupt pin as input

    // Enable One-Shot Transmission mode
    if (CAN.enOneShotTX() == CAN_OK) {
      Serial.println("One-Shot Transmission Enabled!");
    } else {
      Serial.println("Failed to Enable One-Shot Transmission...");
    }
    // Set the MCP2515 to normal mode to allow sending and receiving.
    CAN.setMode(MCP_NORMAL);
    delay(100);
    
    Zero();
    EnterMode();
    Serial.println("CAN BUS Shield init ok!");
}

// Add these new helper functions
void printMenu() {
    Serial.println("\n=== CubeMars AK80-64 Control Menu ===");
    Serial.println("1. Enter MIT Mode");
    Serial.println("2. Exit MIT Mode");
    Serial.println("3. Send Control Command");
    Serial.println("4. Read Status");
    Serial.println("Please enter your choice (1-4):");
}

float readFloatFromSerial(const char* paramName, float minVal, float maxVal) {
    float value;
    Serial.print("Enter ");
    Serial.print(paramName);
    Serial.print(" (");
    Serial.print(minVal);
    Serial.print(" to ");
    Serial.print(maxVal);
    Serial.print("): ");
    
    while (!Serial.available()) {
        delay(100);
    }
    
    value = Serial.parseFloat();
    Serial.println(value);  // Echo the input
    return constrain(value, minVal, maxVal);
}

void sendStatusCommand() {
    // Store original values
    float orig_kp = kp_in;
    float orig_kd = kd_in;
    float orig_t = t_in;
    
    // Zero out control parameters
    kp_in = 0.0f;
    kd_in = 0.0f;
    t_in = 0.0f;
    
    // Send command
    pack_cmd();
    
    // Restore original values
    kp_in = orig_kp;
    kd_in = orig_kd;
    t_in = orig_t;
}

// Replace the existing loop() function with this new one
void loop() {
    printMenu();
    
    while (!Serial.available()) {
        delay(100);
    }
    
    char choice = Serial.read();
    Serial.println(choice);  // Echo the choice
    
    switch (choice) {
        case '1':
            Serial.println("Entering MIT Mode...");
            EnterMode();
            break;
            
        case '2':
            Serial.println("Exiting MIT Mode...");
            ExitMode();
            break;
            
        case '3':
            Serial.println("\nEntering Control Command Parameters:");
            p_in = readFloatFromSerial("position", P_MIN, P_MAX);
            v_in = readFloatFromSerial("velocity", V_MIN, V_MAX);
            kp_in = readFloatFromSerial("kp", KP_MIN, KP_MAX);
            kd_in = readFloatFromSerial("kd", KD_MIN, KD_MAX);
            t_in = readFloatFromSerial("torque", T_MIN, T_MAX);
            
            Serial.println("Sending command...");
            pack_cmd();
            break;
            
        case '4':
            Serial.println("Reading status...");
            sendStatusCommand();
            break;
            
        default:
            Serial.println("Invalid choice! Please select 1-4");
            break;
    }
    
    // Read and display motor response
    if(CAN_MSGAVAIL == CAN.checkReceive()) {
        unpack_reply();
        Serial.println("\nMotor Status:");
        Serial.print("Position: "); Serial.println(p_out);
        Serial.print("Velocity: "); Serial.println(v_out);
        Serial.print("Torque: "); Serial.println(t_out);
    }
    
    // Clear any remaining characters in the serial buffer
    while(Serial.available()) {
        Serial.read();
    }
    
    delay(100);
}

void EnterMode() {
    byte buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    CAN.sendMsgBuf(CONTROLLER_ID, 0, 8, buf);
}

void ExitMode() {
    byte buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    CAN.sendMsgBuf(CONTROLLER_ID, 0, 8, buf);
}

void Zero() {
    byte buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    CAN.sendMsgBuf(CONTROLLER_ID, 0, 8, buf);
}

unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if(bits == 12) {
        pgg = (unsigned int) ((x-offset)*4095.0/span);
    }
    if(bits == 16) {
        pgg = (unsigned int) ((x-offset)*65535.0/span);
    }
    return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    float pgg = 0;
    if(bits == 12) {
        pgg = ((float)x_int)*span/4095.0 + offset;
    }
    if(bits == 16) {
        pgg = ((float)x_int)*span/65535.0 + offset;
    }
    return pgg;
}

void pack_cmd() {
    byte buf[8];
    float p_des = constrain(p_in, P_MIN, P_MAX);
    float v_des = constrain(v_in, V_MIN, V_MAX);
    float kp = constrain(kp_in, KP_MIN, KP_MAX);
    float kd = constrain(kd_in, KD_MIN, KD_MAX);
    float t_ff = constrain(t_in, T_MIN, T_MAX);
    
    unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    
    buf[0] = p_int >> 8;
    buf[1] = p_int & 0xFF;
    buf[2] = v_int >> 4;
    buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    buf[4] = kp_int & 0xFF;
    buf[5] = kd_int >> 4;
    buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    buf[7] = t_int & 0xFF;
    
    CAN.sendMsgBuf(CONTROLLER_ID, 0, 8, buf);
}

void unpack_reply() {
  byte len = 0;
  byte buf[8];
  unsigned long canId;  // Need this for the new library format

  // Changed to match the library's function signature
  CAN.readMsgBuf(&canId, &len, buf);    // Read data,  len: data length, buf: data buf
  
  /// unpack ints from CAN buffer ///
  unsigned int id = buf[0];
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];
  /// convert uints to floats ///
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
}