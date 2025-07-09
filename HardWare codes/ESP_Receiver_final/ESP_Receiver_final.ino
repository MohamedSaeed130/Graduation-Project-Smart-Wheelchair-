#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <driver/mcpwm.h>
#include <LiquidCrystal.h>
/************************************* LCD Configuration *********************************************/
const int rs = 16, en = 17, d4 = 27, d5 = 26, d6 = 25, d7 = 33;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
/************************************* MACROS *********************************************/
// Ultrasonic pins 
#define FRONT_TRIG_PIN 12    // Trigger Pin
#define FRONT_ECHO_PIN 13    // Echo Pin
#define RIGHT_TRIG_PIN 32    // Trigger Pin0
#define RIGHT_ECHO_PIN 35    // Echo Pin
#define LEFT_TRIG_PIN 15     // Trigger Pin
#define LEFT_ECHO_PIN 4      // Echo Pin
#define MOTPR_SPEED_TURN 40  // 

// Motor A
#define MOTOR_A_IN1          23
#define MOTOR_A_IN2          22
#define MOTOR_A_EN_SPEED     21  // PWM output for Motor A

// Motor B
#define MOTOR_B_IN3         19
#define MOTOR_B_IN4         18
#define MOTOR_B_EN_SPEED    5  // PWM output for Motor A

// PWM properties
#define PWM_FREQUENCY  10000            // 5 kHz

/************************************* WIFI Configuration *********************************************/
// Wi-Fi credentials
const char* ssid = "Ahmed"; 
const char* password = "123456789"; 
// UDP settings
WiFiUDP udp;
const unsigned int localPort = 1234; // Port to listen on
char packetBuffer[255];              // Buffer for UDP data
/************************************* DATA TYPES *********************************************/
typedef struct struct_message {
    String mode;
    String motor_direction; 
    uint8_t motor_speed;
    uint8_t speed_shift;
} struct_message;

struct_message received_Data;

String motor_direction="";
String mode_="JOYSTICK";
String last_mode="";
String last_direction="";
uint8_t last_speed=0;
uint8_t last_shift_speed=0;

/************************************* Function Prototypes *********************************************/
void moveForward(uint8_t speed,uint8_t speed_shift_val);
void moveBackward(uint8_t speed,uint8_t speed_shift_val);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();
bool Ultrasonic_func_calc(uint8_t TRIG_PIN,uint8_t ECHO_PIN,float distance_threshold);
void motor_control_func(String direction,uint8_t speed_shift_val,uint8_t m_speed,bool front_ultr_status,bool right_ultr_status, bool left_ultr_status);
void display_mode(String _mode);
void display_Speed(uint8_t speed_);

// ESP-NOW callback function
void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) 
{
  if (len == 0)
  {
    return;
  }
  memcpy(&received_Data, incomingData, sizeof(received_Data));
  mode_=received_Data.mode;
}

void setup() {
  Serial.begin(115200);

  // Configure motor control pins as outputs
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  // Configure Ultrasonic pins 
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Mode:");

  // Configure MCPWM for motor A (PWM on A_EN)
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_A_EN_SPEED);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_B_EN_SPEED);
  mcpwm_config_t pwm_config_motor_A = {
        .frequency = PWM_FREQUENCY, 
        .cmpr_a = 0, // Initial duty cycle 0%
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0, 
        .counter_mode = MCPWM_UP_COUNTER
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_motor_A);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); // Note this IP for the laptop

  // Start UDP
  udp.begin(localPort);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onDataRecv); // Register callback for ESP-NOW
}

void loop() {
  //Serial.println(received_Data.mode);
  // Check for UDP packets
  if(received_Data.mode=="JOYSTICK")
  {
    motor_direction=received_Data.motor_direction;
    mode_="JOYSTICK";
    display_mode(mode_);
  }
  else
  {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0; // Null-terminate the string
      }
      int delimiterIndex = String(packetBuffer).indexOf('|');
      if (delimiterIndex != -1) 
      {
        motor_direction = String(packetBuffer).substring(0, delimiterIndex);
        mode_ = String(packetBuffer).substring(delimiterIndex + 1);
        Serial.print("Received from UDP direction : ");
        Serial.println(motor_direction);
        Serial.print("Received from UDP mode : ");
        Serial.println(mode_);
      }
      Serial.print("Received from UDP: ");
      Serial.println(packetBuffer);
    }
  }
  bool front_ultr_status=Ultrasonic_func_calc(FRONT_TRIG_PIN,FRONT_ECHO_PIN,20);
  bool right_ultr_status=Ultrasonic_func_calc(RIGHT_TRIG_PIN,RIGHT_ECHO_PIN,15);
  bool left_ultr_status=Ultrasonic_func_calc(LEFT_TRIG_PIN,LEFT_ECHO_PIN,15);
  motor_control_func(motor_direction,received_Data.speed_shift,received_Data.motor_speed,front_ultr_status,right_ultr_status,left_ultr_status);
  if(motor_direction != last_direction || mode_ != last_mode || received_Data.motor_speed != last_speed || received_Data.speed_shift != last_shift_speed)
  {
    
    last_direction=motor_direction;
    last_mode=mode_;
    last_speed=received_Data.motor_speed;
    last_shift_speed=received_Data.speed_shift;

    Serial.print("Speed=");
    Serial.println(received_Data.motor_speed);
    Serial.print("Direction=");
    Serial.println(motor_direction);
    Serial.print("mode=");
    Serial.println(mode_);
    display_mode(mode_);
    Serial.println(last_shift_speed);
    display_Speed(received_Data.motor_speed);
  }
  
}

void motor_control_func(String direction,uint8_t speed_shift_val,uint8_t m_speed,bool front_ultr_status,bool right_ultr_status, bool left_ultr_status)
{
  if (direction=="FORWARD" | direction=="MOVE" ) 
  {
    if(front_ultr_status)
      {moveForward(m_speed,speed_shift_val);
      lcd.setCursor(0,1);
      lcd.print("    Forward     ");}
    else
      {stopMotors();
      lcd.setCursor(0,1);
      lcd.print("There is a Block");}
  }
  else if(direction=="RIGHT")
  {
    if(right_ultr_status)
     { turnRight(m_speed);
      lcd.setCursor(0,1);
      lcd.print("   TURN RIGHT   ");}
    else
      {stopMotors();
      lcd.setCursor(0,1); 
      lcd.print("CANNOT TURNRIGHT");}
  }
  else if(direction=="LEFT")
  {
    if(left_ultr_status)
      {turnLeft(m_speed);
      lcd.setCursor(0,1);
      lcd.print("   TURN LEFT    ");}
    else
      {stopMotors();
      lcd.setCursor(0,1);
      lcd.print("CANNOT TURNLEFT ");}
    
  }
  else if(direction=="BACK" || direction=="BACKWARD" )
  {
    moveBackward(m_speed,speed_shift_val);
    lcd.setCursor(0,1);
    lcd.print("     BACK       ");
  }
  else if(direction=="STOP")
  {
    stopMotors();
    lcd.setCursor(0,1);
    lcd.print("     STOP       ");
  }
  else
  {}
}


// Function to move forward
void moveForward(uint8_t speed,uint8_t speed_shift_val) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, HIGH);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed+received_Data.speed_shift);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed);
    delay(20);
    display_Speed(speed);
}

// Function to move backward
void moveBackward(uint8_t speed,uint8_t speed_shift_val) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN3, HIGH);
    digitalWrite(MOTOR_B_IN4, LOW);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed+received_Data.speed_shift);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed);
    delay(20);
    display_Speed(speed);
}

// Function to turn left (stop right motor, move left motor forward)
void turnLeft(int speed) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, LOW);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speed);
    display_Speed(speed);
}

// Function to turn right (stop left motor, move right motor forward)
void turnRight(int speed) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, HIGH);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, speed);
    display_Speed(speed);
}

// Function to stop both motors
void stopMotors() {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN3, LOW);
    digitalWrite(MOTOR_B_IN4, LOW);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 0);
    delay(20);
}

bool Ultrasonic_func_calc(uint8_t TRIG_PIN,uint8_t ECHO_PIN,float distance_threshold)
{
  long duration;
  float distance;
    // Send a 10µs HIGH pulse to trigger measurement
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Measure the echo duration
  duration= pulseIn(ECHO_PIN, HIGH);
  // Calculate distance in cm (Speed of sound = 343m/s or 0.0343 cm/µs)
  distance = (duration * 0.0343) / 2;
  delay(5);

  if(distance > distance_threshold)
    return true;
  else
    return false;
}

void display_mode(String _mode)
{
  lcd.setCursor(0,0);
  lcd.print("Mode:");
  lcd.setCursor(5,0);
  lcd.print("        ");
  lcd.setCursor(5,0);
  lcd.print(_mode);
}

void display_Speed(uint8_t speed_)
{
  lcd.setCursor(14,0);
  lcd.print("  ");
  lcd.setCursor(13,0);
  lcd.print(speed_);
}

