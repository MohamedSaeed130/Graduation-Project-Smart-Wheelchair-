#include <esp_now.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin definitions
#define X_AXIS                  34
#define Y_AXIS                  35
#define MOTOR_SPEED_CONTROL_POT 32
#define MOTOR_SPEED_SHIFT_POT   33
#define buttonPin               13
#define TEMPERATURE_SENSOR_PIN  25

// —— WiFi + UDP globals ——
const char* ssid        = "MohammedAbuSamaha";
const char* password    = "Hamo.98130";
const uint16_t laptopPort = 5006;
WiFiUDP Udp;

// —— ESP‑NOW globals ——
// Two receiver MAC addresses
uint8_t broadcastAddress1[] = {0x40, 0x22, 0xD8, 0x4E, 0xF4, 0x28};
uint8_t broadcastAddress2[] = {0xEC, 0x62, 0x60, 0x75, 0xF8, 0x60};

typedef struct struct_message {
    String mode;
    String motor_direction;
    uint8_t motor_speed;
    uint8_t speed_shift;
} espnow_message;

PulseOximeter pox;
OneWire oneWire(TEMPERATURE_SENSOR_PIN);
DallasTemperature temperature_sensor(&oneWire);
String mode_select = "JOYSTICK";
volatile unsigned long lastPress = 0;
const unsigned long debounceDelay = 200;
uint16_t x_axis_value = 0;
uint16_t y_axis_value = 0;

uint8_t last_speed = 0;
String last_mode = "";
String last_direction = "";
uint8_t last_shift_speed = 0;
unsigned long lastReportTime = 0;

uint32_t lastUpdate = 0;
unsigned long lastUdp = 0;
const unsigned long udpInterval = 1000; // 1s

// Interrupt for button toggle
void IRAM_ATTR ISR_function() {
    unsigned long now = millis();
    if (now - lastPress > debounceDelay) {
        mode_select = (mode_select != "JOYSTICK") ? "JOYSTICK" : "OTHER";
        lastPress = now;
    }
}

// Callback for ESP-NOW send status
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Data sent to ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac_addr[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.print(" - Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Map joystick to direction
String Map_Function(uint16_t X_value, uint16_t Y_value) {
    if (X_value > 3000) return "FORWARD";
    else if (X_value < 100) return "BACK";
    else if (Y_value > 3000) return "RIGHT";
    else if (Y_value < 100) return "LEFT";
    else return "STOP";
}

// Send data to both ESP32s via ESP-NOW
void sendToAllReceivers(espnow_message &data) {
    // Send to first ESP32
    esp_err_t result1 = esp_now_send(broadcastAddress1, (uint8_t *)&data, sizeof(data));
    if (result1 == ESP_OK) {
        Serial.println("ESP-NOW data sent to ESP32 #1");
    } else {
        Serial.println("Error sending data to ESP32 #1");
    }
    
    // Send to second ESP32
    esp_err_t result2 = esp_now_send(broadcastAddress2, (uint8_t *)&data, sizeof(data));
    if (result2 == ESP_OK) {
        Serial.println("ESP-NOW data sent to ESP32 #2");
    } else {
        Serial.println("Error sending data to ESP32 #2");
    }
}

// Send combined UDP packet
void sendUdpStatus(float tempC, float tempF, float avgHr,
                   const String &mode, const String &dir,
                   uint8_t speed, uint8_t shift) {
    char buf[128];

    int len = snprintf(buf, sizeof(buf),
      "{\"temperature\":%.2f, \"heartRate\":%.1f, \"bloodOxygen\": %d}",
      tempC, avgHr, speed
    );

    Udp.beginPacket(WiFi.broadcastIP(), laptopPort);
    Udp.write((uint8_t*)buf, len);
    Udp.endPacket();
    Serial.println("UDP → " + String(buf));
}

void setup() {
    Serial.begin(115200);

    // 1) Wi‑Fi for UDP
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting Wi‑Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.println();
    Serial.print("Wi‑Fi ready, IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Broadcast IP: ");
    Serial.println(WiFi.broadcastIP()); // Print broadcast IP for verification
    Udp.begin(0);

    // 2) ESP‑NOW init
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        while(1);
    }
    
    // Add first peer (ESP32 #1)
    esp_now_peer_info_t peer1 = {};
    memcpy(peer1.peer_addr, broadcastAddress1, 6);
    peer1.channel = 0;
    peer1.encrypt = false;
    if (esp_now_add_peer(&peer1) != ESP_OK) {
        Serial.println("Failed to add ESP32 #1 peer");
    } else {
        Serial.println("ESP32 #1 peer added successfully");
    }
    
    // Add second peer (ESP32 #2)
    esp_now_peer_info_t peer2 = {};
    memcpy(peer2.peer_addr, broadcastAddress2, 6);
    peer2.channel = 0;
    peer2.encrypt = false;
    if (esp_now_add_peer(&peer2) != ESP_OK) {
        Serial.println("Failed to add ESP32 #2 peer");
    } else {
        Serial.println("ESP32 #2 peer added successfully");
    }
    
    esp_now_register_send_cb(OnDataSent);

    // 3) Sensor setup
    temperature_sensor.begin();
    analogSetPinAttenuation(MOTOR_SPEED_CONTROL_POT, ADC_11db);
    analogSetPinAttenuation(MOTOR_SPEED_SHIFT_POT, ADC_11db);
    pinMode(buttonPin, INPUT_PULLUP);
    attachInterrupt(buttonPin, ISR_function, FALLING);

}

void loop() {

    uint16_t speed_shift_pot = analogRead(MOTOR_SPEED_SHIFT_POT);
    x_axis_value = analogRead(X_AXIS);
    y_axis_value = analogRead(Y_AXIS);
    int pot_read = analogRead(MOTOR_SPEED_CONTROL_POT);
    uint8_t motor_speed_transmit = map(pot_read, 0, 4095, 0, 100);
    String direction = Map_Function(x_axis_value, y_axis_value);
    uint8_t shift = map(speed_shift_pot, 0, 4095, 0, 20);

    // 3) ESP‑NOW send if changed - now sends to both receivers
    espnow_message data_send = { mode_select, direction, motor_speed_transmit, shift };
    if (motor_speed_transmit != last_speed || direction != last_direction ||
        mode_select != last_mode || shift != last_shift_speed) {
        
        sendToAllReceivers(data_send);
        
        last_speed = motor_speed_transmit;
        last_direction = direction;
        last_mode = mode_select;
        last_shift_speed = shift;
    } 
    delay(10); // slight delay to stabilize readings
    // 1) Read sensors
    temperature_sensor.requestTemperatures();
    float c = temperature_sensor.getTempCByIndex(0);
    float f = temperature_sensor.toFahrenheit(c);
    pox.update();
     // Accumulate heart rate
    static uint32_t count = 0;
    static float hrSum = 0;
    float hr = pox.getHeartRate();
    if (hr > 0) { hrSum += hr; count++; }
    float avgHr = (count > 0) ? hrSum/count : 0;
    hrSum = 0; count = 0;
    if (millis() - lastReportTime > 10000)
    {
      sendUdpStatus(c, f, avgHr, mode_select, direction, motor_speed_transmit, shift);
      lastReportTime=millis();
    }
    delay(10);
}
