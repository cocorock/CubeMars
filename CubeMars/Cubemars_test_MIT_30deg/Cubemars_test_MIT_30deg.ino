#include <ACAN2515.h>
#include "RMDX.h"

// Motor IDs
unsigned char motorID1 = 104; // Example motor ID

// Configure CAN bus adapter
static const byte MCP2515_CS = 5;  // CS input of MCP2515
static const byte MCP2515_INT = 17; // INT output of MCP2515
ACAN2515 can(MCP2515_CS, SPI, MCP2515_INT);

// Select the quartz frequency - 8MHz or 16MHz
static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL; // 8 MHz
// CAN frequency: 1Mbps as per manual specification
static const uint32_t CAN_BAUDRATE = 500UL * 1000UL; // 1Mbps CAN

RMDX motor(&can);

void setup() {
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
        Serial.print("Error code: ");
        Serial.println(errorCode);
        while (true);
    }

    // Send special code to enter Motor Control Mode
    CANMessage enterControlMode;
    enterControlMode.id = motorID1;
    enterControlMode.len = 8;
    enterControlMode.data[0] = 0xFF;
    enterControlMode.data[1] = 0xFF;
    enterControlMode.data[2] = 0xFF;
    enterControlMode.data[3] = 0xFF;
    enterControlMode.data[4] = 0xFF;
    enterControlMode.data[5] = 0xFF;
    enterControlMode.data[6] = 0xFF;
    enterControlMode.data[7] = 0xFC;
    can.tryToSend(enterControlMode);

    delay(1000);

    // Command motor to move 30 degrees (≈ 0.523599 radians)
    sendPositionCommand(motorID1, 0.523599);
    Serial.println("sent");
}

void loop() {
    // Process received CAN messages
    CANMessage frame;
    if (can.receive(frame)) {
        processMotorResponse(frame);
    }
}

// Updated position command function following MIT mode protocol
void sendPositionCommand(uint8_t motorID, float positionRad) {
    // Parameter limits from manual
    const float P_MIN = -12.5f;
    const float P_MAX = 12.5f;
    const float V_MIN = -50.0f;
    const float V_MAX = 50.0f;
    const float KP_MIN = 0.0f;
    const float KP_MAX = 500.0f;
    const float KD_MIN = 0.0f;
    const float KD_MAX = 5.0f;

    // Example values - adjust as needed
    float desiredVelocity = 0.0f;
    float kp = 100.0f;
    float kd = 2.0f;
    float torque = 0.0f;

    CANMessage msg;
    msg.id = motorID;
    msg.len = 8;

    // Convert floats to ints using the protocol's conversion method
    int p_int = float_to_uint(positionRad, P_MIN, P_MAX, 16);
    int v_int = float_to_uint(desiredVelocity, V_MIN, V_MAX, 12);
    int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    int t_int = float_to_uint(torque, -18.0f, 18.0f, 12);

    // Pack data according to MIT mode protocol
    msg.data[0] = p_int >> 8;
    msg.data[1] = p_int & 0xFF;
    msg.data[2] = v_int >> 4;
    msg.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg.data[4] = kp_int & 0xFF;
    msg.data[5] = kd_int >> 4;
    msg.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg.data[7] = t_int & 0xFF;

    can.tryToSend(msg);
}

// Helper function for float to uint conversion
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
    float span = x_max - x_min;
    if(x < x_min) x = x_min;
    else if(x > x_max) x = x_max;
    return (int)((x - x_min) * ((float)((1 << bits) - 1) / span));
}

// Process motor response
void processMotorResponse(CANMessage &msg) {
    if(msg.len == 8) {
        int id = msg.data[0];
        int p_int = (msg.data[1] << 8) | msg.data[2];
        int v_int = (msg.data[3] << 4) | (msg.data[4] >> 4);
        int i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5];
        int temp = msg.data[6];

        // Convert to actual values
        float position = uint_to_float(p_int, -12.5f, 12.5f, 16);
        float velocity = uint_to_float(v_int, -50.0f, 50.0f, 12);
        float current = uint_to_float(i_int, -18.0f, 18.0f, 12);
        float temperature = temp - 40; // Temperature range: -40~215

        // Print motor status
        Serial.print("Motor ID: "); Serial.println(id);
        Serial.print("Position: "); Serial.println(position);
        Serial.print("Velocity: "); Serial.println(velocity);
        Serial.print("Current: "); Serial.println(current);
        Serial.print("Temperature: "); Serial.println(temperature);
    }
}

// Helper function for uint to float conversion
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
