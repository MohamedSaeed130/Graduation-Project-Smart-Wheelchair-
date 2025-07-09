#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>

#define TURN_SPEED  30

/************************************* WIFI Configuration *********************************************/
// Wi-Fi credentials
const char* ssid = "Ahmed"; 
const char* password = "123456789"; 
// UDP settings
WiFiUDP udp;
const unsigned int localPort = 1234; // Port to listen on
char packetBuffer[255];              // Buffer for UDP data

/************************************* MOTOR PINS *********************************************/
//--- Motor 1 (Left Motor) ---//
const int M1_EN_pin = 32;   // Enable pin for Motor 1
const int M1_RPWM_pin = 25; // MCPWM0A → Motor 1 forward
const int M1_LPWM_pin = 26; // MCPWM0B → Motor 1 reverse

//--- Motor 2 (Right Motor) ---//
const int M2_EN_pin = 33;   // Enable pin for Motor 2
const int M2_RPWM_pin = 14; // MCPWM1A → Motor 2 forward
const int M2_LPWM_pin = 27; // MCPWM1B → Motor 2 reverse

/************************************* DATA TYPES *********************************************/
typedef struct struct_message {
    String mode;
    String motor_direction; 
    uint8_t motor_speed;
} struct_message;

struct_message received_Data;

String motor_direction="";
String mode_="JOYSTICK";
String last_mode="";
String last_direction="";
uint8_t last_speed=0;
uint8_t current_speed = 0;

/************************************* ESP-NOW Communication *********************************************/
// ESP-NOW callback function
void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&received_Data, incomingData, sizeof(received_Data));
  
  /*Serial.println("ESP-NOW Data Received:");
  Serial.print("Mode: ");
  Serial.println(received_Data.mode);
  Serial.print("Direction: ");
  Serial.println(received_Data.motor_direction);
  Serial.print("Speed: ");
  Serial.println(received_Data.motor_speed);*/
  current_speed=received_Data.motor_speed;
}

/************************************* MOTOR CONTROL FUNCTIONS *********************************************/
void initializeMotors() {
  // MCPWM GPIO init
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, M1_RPWM_pin);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, M1_LPWM_pin);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, M2_RPWM_pin);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, M2_LPWM_pin);

  // Configure enable pins
  pinMode(M1_EN_pin, OUTPUT);
  pinMode(M2_EN_pin, OUTPUT);
  digitalWrite(M1_EN_pin, HIGH); // Enable Motor 1
  digitalWrite(M2_EN_pin, HIGH); // Enable Motor 2

  // Configure MCPWM parameters for both units
  mcpwm_config_t cfg;
  cfg.frequency = 5000;           // 5 kHz
  cfg.cmpr_a = 0.0;              // Initial duty for operator A
  cfg.cmpr_b = 0.0;              // Initial duty for operator B
  cfg.counter_mode = MCPWM_UP_COUNTER;
  cfg.duty_mode = MCPWM_DUTY_MODE_0;

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg); // Motor 1
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &cfg); // Motor 2
  
  Serial.println("Motors initialized successfully");
}

void setMotorSpeed(mcpwm_unit_t unit, mcpwm_timer_t timer, int speed, bool forward) {
  // Speed is already 0-100, so just constrain it
  int duty = constrain(speed, 0, 100);
  Serial.println(duty);
  if (forward) {
    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, duty);  // Forward
    mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 0);     // Stop reverse
  } else {
    mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 0);     // Stop forward
    mcpwm_set_duty(unit, timer, MCPWM_OPR_B, duty);  // Reverse
  }
  
  // Apply duty changes immediately
  mcpwm_set_duty_type(unit, timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(unit, timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void stopMotor(mcpwm_unit_t unit, mcpwm_timer_t timer) {
  mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 0);
  mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 0);
  mcpwm_set_duty_type(unit, timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(unit, timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}

void moveForward(uint8_t speed) {
  Serial.printf("Moving Forward at speed: %d\n", speed);
  setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, speed, true);  // Left motor forward
  setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, speed, true);  // Right motor forward
}

void moveBackward(uint8_t speed) {
  Serial.printf("Moving Backward at speed: %d\n", speed);
  setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, speed, false); // Left motor reverse
  setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, speed, false); // Right motor reverse
}

void turnLeft(uint8_t speed) {
  Serial.printf("Turning Left at speed: %d\n", speed);
  setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, false); // Left motor reverse
  setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, speed, true);  // Right motor forward
}

void turnRight(uint8_t speed) {
  Serial.printf("Turning Right at speed: %d\n", speed);
  setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, speed, true);  // Left motor forward
  setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, 0, false); // Right motor reverse
}

void stopAllMotors() {
  //Serial.println("Stopping all motors");
  stopMotor(MCPWM_UNIT_0, MCPWM_TIMER_0); // Stop left motor
  stopMotor(MCPWM_UNIT_1, MCPWM_TIMER_1); // Stop right motor
}

void executeMotorCommand(String direction, uint8_t speed) {
  if (speed == 0) {
    stopAllMotors();
    return;
  }
  if (direction == "FORWARD" || direction == "MOVE") {
    moveForward(speed);
  }
  else if (direction == "BACK" || direction == "DOWN") {
    moveBackward(speed);
  }
  else if (direction == "LEFT") {
    turnLeft(TURN_SPEED);
  }
  else if (direction == "RIGHT") {
    turnRight(TURN_SPEED);
  }
  else if (direction == "STOP") {
    stopAllMotors();
  }
  else {
    Serial.println("Unknown direction command: " + direction);
    stopAllMotors();
  }
}

String current_direction_UDP="";

/************************************* SETUP *********************************************/
void setup() {
  Serial.begin(115200);
  
  // Initialize motors first
  initializeMotors();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start UDP
  udp.begin(localPort);
  Serial.print("UDP server started on port: ");
  Serial.println(localPort);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onDataRecv);
  Serial.println("ESP-NOW initialized successfully");
  
  Serial.println("System ready - waiting for commands...");
}

/************************************* MAIN LOOP *********************************************/
void loop() {
  
  String current_direction = "";
  
  // Handle ESP-NOW received data (priority over UDP)
  if(received_Data.mode == "JOYSTICK")
  {
    current_direction = received_Data.motor_direction;
    current_speed = received_Data.motor_speed;
    mode_ = "JOYSTICK";
    
    // Execute motor command immediately for joystick mode
    executeMotorCommand(current_direction, current_speed);
  }
  else
  {
    // Handle UDP communication
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0; // Null-terminate the string
      }
      
      // Parse UDP data (format: "direction|mode|speed")
      String udpData = String(packetBuffer);
      int delimiterIndex = udpData.indexOf('|');
      
      if (delimiterIndex != -1) 
      {
        current_direction_UDP = String(packetBuffer).substring(0, delimiterIndex);
        mode_ = String(packetBuffer).substring(delimiterIndex + 1);
        
        /*Serial.print("Received from UDP - Direction: ");
        Serial.print(current_direction_UDP);
        Serial.print(", Mode: ");
        Serial.print(mode_);
        Serial.print(", Speed: ");
        Serial.println(current_speed);*/
        
        // Execute motor command for UDP mode
        executeMotorCommand(current_direction_UDP, current_speed);
      }
    }
  }
  
  // Check for data changes and print status
  if(current_direction != last_direction || mode_ != last_mode || 
     received_Data.motor_speed != last_speed)
  {
    last_direction = current_direction;
    last_mode = mode_;
    last_speed = received_Data.motor_speed;

    Serial.println("=== Status Update ===");
    Serial.print("Mode: ");
    Serial.println(mode_);
    Serial.print("Direction: ");
    Serial.println(current_direction);
    Serial.print("Speed: ");
    Serial.println(current_speed);
    Serial.println("====================");
  }
  
  delay(10); // Small delay to prevent overwhelming the system
}