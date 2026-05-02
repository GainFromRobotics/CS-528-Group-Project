#include <Arduino.h>

#include "rplidarc1.h"
// Let's test our my library

#define TEST_MY_LIBRARY

#define RPLIDAR_RX GPIO_NUM_16  // ESP32 Pin connected to LiDAR TX
#define RPLIDAR_TX GPIO_NUM_17 // ESP32 Pin connected to LiDAR RX

#ifdef TEST_MY_LIBRARY
RPLidarC1 rplidarc1(UART_DEFAULT,RPLIDAR_RX,RPLIDAR_TX,BAUD_RATE_DEFAULT);
#else
HardwareSerial LidarSerial(2);
#endif

void setup() {
  
  Serial.begin(115200); // USB Debug Monitor

#ifdef TEST_MY_LIBRARY
  rplidarc1.begin();
#else
  LidarSerial.begin(460800, SERIAL_8N1, RPLIDAR_RX, RPLIDAR_TX);
#endif
  
  delay(3000);
  Serial.println("ESP32 RPLIDAR C1 Handshake Test");
  
#ifdef TEST_MY_LIBRARY
  //delay(1000);

  // First test, manually perform the actions to get info of our lidar
  device_info dev_I = rplidarc1.cmd_get_info();
  printf("device_info obtained! Let's print it\n");
  char tempbuf[256];
  printf("Device Info is %s\n",to_string(dev_I,tempbuf,256));

  delay(500);

  // Test get device health
  device_health dev_H = rplidarc1.cmd_get_health();
  printf("device health is %s\n",to_string(dev_H,tempbuf,256));

  delay(500);

  // Test get sample rate
  device_sample_rate dev_SR = rplidarc1.cmd_get_samplerate();
  printf("Device sample rate is %s\n",to_string(dev_SR,tempbuf,256));

  //delay(3000); //seems like enough time to wait for the thing to speed up

  // Test standard scan
  response_descriptor scan_rd = rplidarc1.cmd_scan();
  for(int i = 0; i < 500; i++) {
    scan_standard_sample sss = rplidarc1.get_latest_scan(scan_rd);
    printf("CURRENT SCAN: %s\n",to_string(sss,tempbuf,256));
    //delay(20); //idk seems reasonable
  }
  
  // test stop
  rplidarc1.cmd_stop();

  delay(1000);

  // reset everything
  rplidarc1.cmd_reset();

#else
  //Command: 0xA5 0x25
  byte getInfoCmd[] = {0xA5, 0x50};
  LidarSerial.write(getInfoCmd, 2);
  Serial.println("Sent GET_INFO command. Waiting for response...");
#endif


  
  
  // Second test, does this return flawlessly???
  //device_sample_rate dev_sr = rplidarc1.cmd_get_samplerate();
  //char buf[256];
  //printf("%s",to_string(dev_sr,buf,256));

  
}

void loop() {

#ifdef TEST_MY_LIBRARY
  // When we get our scan data we'll need to eventually render it to our external renderer
#else
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
#endif
}