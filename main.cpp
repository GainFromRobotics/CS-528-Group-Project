#include <Arduino.h>
// Define UART pins (Using UART2)
#define RPLIDAR_RX 4  // ESP32 Pin connected to LiDAR TX
#define RPLIDAR_TX 5  // ESP32 Pin connected to LiDAR RX

HardwareSerial LidarSerial(2); // Use UART2

void setup() {
  Serial.begin(115200); // USB Debug Monitor
  LidarSerial.begin(460800, SERIAL_8N1, RPLIDAR_RX, RPLIDAR_TX);
  
  delay(3000);
  Serial.println("ESP32 RPLIDAR C1 Handshake Test");
  
  // Send STOP command (Standard RPLIDAR Protocol)
  // Command: 0xA5 0x25
  byte getInfoCmd[] = {0xA5, 0x50};
  LidarSerial.write(getInfoCmd, 2);
  
  Serial.println("Sent GET_INFO command. Waiting for response...");
}

void loop() {
  // Read and print raw hex data coming from LiDAR
  if (LidarSerial.available()) {
    int incomingByte = LidarSerial.read();
    Serial.print("Received: 0x");
    Serial.println(incomingByte, HEX);
  }
  
  // Also report if nothing is being received
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 2000) {
    lastCheck = millis();
    if (!LidarSerial.available()) {
      Serial.println("Waiting... (No data yet)");
    }
  }

}