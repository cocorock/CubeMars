/*
SERVO MODE - SERIAL TERMINAL USAGE INSTRUCTIONS:

0.One IMPORTANT thing is, when you decide MIT or Servo mode, you
  must change mode ,firstly, via R-Link, otherwise drive card can be
  burned. Another IMPORTANT thing is, the first time you use Servo
  mode, you should calibrate on application function via R-link, if you
  don’t, can bus cannot work.

1. Connect to the Arduino through serial terminal at 115200 baud rate

2. Available Commands:
   - 'x': Increase motor speed by 1500 RPM
   - 's': Decrease motor speed by 1500 RPM

3. Serial Output:
   - Displays the command received
   - Shows "RPM increased" or "RPM decreased" messages
   - Shows CAN initialization status at startup

4. Initial Setup:
   - Program automatically initializes CAN bus at 500kbps
   - Wait for "CAN init ok!" message before sending commands

5. Hardware Configuration:
   - MCP2515 CS Pin: 5
   - MCP2515 INT Pin: 17
   - CAN Speed: 500kbps
   - Crystal: 8MHz

6. Motor Control Limits:
   - Maximum PWM: 10000
   - Maximum Current: 60000 mA
   - Maximum Velocity: 10000 RPM
   - Maximum Position: 360000
   - Position Velocity Range: -32768 to 32767
   - Maximum Acceleration: 40000
*/

//Required libraries
#include <mcp_can.h>
#include <SPI.h>

// Pin Definitions
#define MCP2515_CS   5    // Chip Select pin for MCP2515
#define MCP2515_INT  17   // Interrupt pin from MCP2515

// Initialize MCP_CAN instance
MCP_CAN CAN(MCP2515_CS);

// Some constants <<<-------------------===============================================
const float MAX_PWM = 10000.0f;
const float MAX_CURRENT = 60000.0f;
const float MAX_VELOCITY = 10000.0f;
const float MAX_POSITION = 360000.0f;
const float MAX_POSITION_VELOCITY = 3276700.0f;
const float MIN_POSITION_VELOCITY = -327680.0f;
const float MAX_ACCELERATION = 40000.0f;

enum AKMode {
  AK_PWM = 0,
  AK_CURRENT,
  AK_CURRENT_BRAKE,
  AK_VELOCITY,
  AK_POSITION,
  AK_ORIGIN,
  AK_POSITION_VELOCITY,
};

const int NUM_MODES = 7;
const String MENU_OPTIONS[] = {
  "Set PWM Duty",
  "Set Current",
  "Set Current Brake",
  "Set RPM (Velocity)",
  "Set Position",
  "Set Origin",
  "Set Position with Speed"
};

uint32_t canId(int id, AKMode Mode_set) {
  uint32_t mode;
  mode = Mode_set;
  return uint32_t(id | mode << 8);
}

void comm_can_transmit_eid(uint32_t id, const uint8_t* data, uint8_t len) {
  uint8_t i = 0;
  if (len > 8) {
    len = 8;
  }
  uint8_t buf[len];
  for (i = 0; i < len; i++) {
    buf[i] = data[i];
  }
  CAN.sendMsgBuf(id, 1, len, buf); //Extended frame format
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t* index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t* index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void comm_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 10000.0), &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_PWM), buffer, send_index);
}

void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_CURRENT), buffer, send_index);
}

void comm_can_set_cb(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_CURRENT_BRAKE), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_VELOCITY), buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_POSITION), buffer, send_index);
}

void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)set_origin_mode, &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_ORIGIN), buffer, send_index);
}

void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA) {
  int32_t send_index = 0;
  int16_t send_index1 = 4;
  uint8_t buffer[8];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(buffer, spd, &send_index1);
  buffer_append_int16(buffer, RPA, &send_index1);
  
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_POSITION_VELOCITY), buffer, send_index);
}

void printMotorData(float pos, float spd, float cur, int8_t temp, int8_t error) {
    Serial.print("Pos: ");
    Serial.print(pos, 2);  // 2 decimal places
    Serial.print("°,");
    
    Serial.print(" Spd: ");
    Serial.print(spd, 2);
    Serial.print(" RPM,");
    
    Serial.print(" I: ");
    Serial.print(cur, 2);
    Serial.print(" A,");
    
    Serial.print(" Temp: ");
    Serial.print(temp);
    Serial.print("°C,");
    
    Serial.print(" Error: ");
    Serial.print(error);
    Serial.println(",");
}

void printMotorDataArduinoF(float pos, float spd, float cur, int8_t temp, int8_t error) {
    // Format: position,speed,current,temperature,error
    Serial.print(pos);
    Serial.print(",");
    Serial.print(spd);
    Serial.print(",");
    Serial.print(cur);
    Serial.print(",");
    Serial.print(temp);
    Serial.print(",");
    Serial.println(error);
}

void motor_receive(float* motor_pos, float* motor_spd, float* motor_cur, int8_t* motor_temp, int8_t* motor_error, uint8_t* rx_message) {
    byte len = 0;
    byte buf[8];
    unsigned long canId;
    
    if(CAN_MSGAVAIL == CAN.checkReceive()) {
        CAN.readMsgBuf(&canId, &len, buf);    // Read data according to coryjfowler library
        
        int16_t pos_int = buf[0] << 8 | buf[1];
        int16_t spd_int = buf[2] << 8 | buf[3];
        int16_t cur_int = buf[4] << 8 | buf[5];
        *motor_pos = (float)(pos_int * 0.1f);
        *motor_spd = (float)(spd_int * 10.0f);
        *motor_cur = (float)(cur_int * 0.01f);
        *motor_temp = buf[6];
        *motor_error = buf[7];
        
        // Print the received data in a nice format
        printMotorDataArduinoF(*motor_pos, *motor_spd, *motor_cur, *motor_temp, *motor_error);
    }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  
  pinMode(MCP2515_INT, INPUT);  // Setting interrupt pin as input
  
  // Initialize MCP2515 with new parameters
  while (CAN_OK != CAN.begin(MCP_STDEXT, CAN_1000KBPS, MCP_16MHZ)) {
    Serial.println("CAN BUS Shield init fail");
    Serial.println("Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN init ok!");

  // Enable One-Shot Transmission mode
  // if (CAN.enOneShotTX() == CAN_OK) {
  //   Serial.println("One-Shot Transmission Enabled!");
  // } else {
  //   Serial.println("Failed to Enable One-Shot Transmission...");
  // }
  
  // Set the MCP2515 to normal mode to allow sending and receiving.
  CAN.setMode(MCP_NORMAL);
  delay(100);

}


void printMenu() {
    Serial.println("\n=== Motor Control Menu ===");
    for (int i = 0; i < NUM_MODES; i++) {
        Serial.print(i + 1);
        Serial.print(". ");
        Serial.println(MENU_OPTIONS[i]);
    }
    Serial.println("Enter your choice (1-7):");
}

float getFloatInput(const char* prompt) {
    Serial.println(prompt);
    while (!Serial.available()) {}
    return Serial.parseFloat();
}

int getIntInput(const char* prompt) {
    Serial.println(prompt);
    while (!Serial.available()) {}
    return Serial.parseInt();
}


void handleMenuChoice(int choice) {
    // Declare all variables at the beginning of the function
    float value = 0;
    float pos = 0;
    int16_t spd = 0;
    int16_t rpa = 0;
    int intValue = 0;
    
    switch(choice) {
        case 1: // PWM Duty
            value = getFloatInput("Enter PWM duty cycle (-1.0 to 1.0):");
            comm_can_set_duty(0xA, value);
            break;
        
        case 2: // Current
            value = getFloatInput("Enter current in Amps:");
            comm_can_set_current(0xA, value);
            break;
        
        case 3: // Current Brake
            value = getFloatInput("Enter brake current in Amps:");
            comm_can_set_cb(0xA, value);
            break;
        
        case 4: // RPM
            value = getFloatInput("Enter RPM:");
            comm_can_set_rpm(0xA, value);
            break;
        
        case 5: // Position
            value = getFloatInput("Enter position in degrees:");
            comm_can_set_pos(0xA, value);
            break;
        
        case 6: // Origin
            intValue = getIntInput("Enter origin mode (0/1):");
            comm_can_set_origin(0xA, intValue);
            break;
        
        case 7: // Position with Speed
            pos = getFloatInput("Enter position in degrees:");
            spd = getIntInput("Enter speed:");
            rpa = getIntInput("Enter RPA:");
            comm_can_set_pos_spd(0xA, pos, spd, rpa);
            break;
        
        default:
            Serial.println("Invalid choice!");
            break;
    }
}

void loop() {
    float motor_pos, motor_spd, motor_cur;
    int8_t motor_temp, motor_error;
    uint8_t rx_message[8];

    if (Serial.available()) {
        printMenu();
        int choice = getIntInput("");
        handleMenuChoice(choice);
        delay(100);
    }
    
    // Read and display motor data
    motor_receive(&motor_pos, &motor_spd, &motor_cur, &motor_temp, &motor_error, rx_message);
    delay(100);
}