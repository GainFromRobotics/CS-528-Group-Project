#include <Arduino.h>
#include "rplidarc1.h"

#include "oled.h"

// Let's test our my library

#define TEST_MY_LIBRARY
//#define TEST_SERIAL
//#define TEST_MPU

#define RPLIDAR_RX GPIO_NUM_16  // ESP32 Pin connected to LiDAR TX
#define RPLIDAR_TX GPIO_NUM_17 // ESP32 Pin connected to LiDAR RX

#ifdef TEST_MY_LIBRARY
RPLidarC1 rplidarc1(UART_DEFAULT,RPLIDAR_RX,RPLIDAR_TX,BAUD_RATE_DEFAULT);
response_descriptor scan_standard_rd;
#endif
#ifdef TEST_SERIAL
HardwareSerial LidarSerial(2);
#endif

void setup() {
  
  Serial.begin(921600); // USB Debug Monitor
  while (!Serial);

  // prepare for the OLED
  setupOLED();

#ifdef TEST_MY_LIBRARY
  Serial.println("Beginning lidar UART communication...");
  rplidarc1.begin();
#endif
#ifdef TEST_SERIAL
  LidarSerial.begin(460800, SERIAL_8N1, RPLIDAR_RX, RPLIDAR_TX);
#endif
  
  delay(3000);
  //Serial.println("ESP32 RPLIDAR C1 Handshake Test");
  
#ifdef TEST_MY_LIBRARY
  //delay(1000);

  // First test, manually perform the actions to get info of our lidar
  device_info dev_I = rplidarc1.cmd_get_info();
  char tempbuf[256];
  Serial.printf("Device Info is %s\n",to_string(dev_I,tempbuf,256));

  delay(500);

  // Test get device health
  device_health dev_H = rplidarc1.cmd_get_health();
  Serial.printf("device health is %s\n",to_string(dev_H,tempbuf,256));

  delay(500);

  // Test get sample rate
  device_sample_rate dev_SR = rplidarc1.cmd_get_samplerate();
  Serial.printf("Device sample rate is %s\n",to_string(dev_SR,tempbuf,256));

  delay(300); //seems like enough time to wait for the thing to speed up

  // Test standard scan
  //response_descriptor scan_rd = rplidarc1.cmd_scan();
  //for(int i = 0; i < 500; i++) {
  //  scan_standard_sample sss = rplidarc1.get_latest_scan(scan_rd);
  //  printf("CURRENT SCAN: %s\n",to_string(sss,tempbuf,256));
  //  //delay(20); //idk seems reasonable
  //}

  scan_standard_rd = rplidarc1.cmd_scan(); // you are not being received, what gives?
  Serial.printf("Scan Response Descriptor: %s\n",to_string(scan_standard_rd,tempbuf,256));
  
  // test stop
  //rplidarc1.cmd_stop();

  //delay(1000);

  // reset everything
  //rplidarc1.cmd_reset();
#endif
#ifdef TEST_SERIAL
  //Command: 0xA5 0x25
  byte getInfoCmd[] = {0xA5, 0x50};
  LidarSerial.write(getInfoCmd, 2);
  Serial.println("Sent GET_INFO command. Waiting for response...");
#endif
  
}

void loop() {

#ifdef TEST_MY_LIBRARY

  //rplidarc1.printUartErrors();

  // When we get our scan data we'll need to eventually render it to our external renderer
  scan_standard_sample sample = rplidarc1.get_latest_scan(scan_standard_rd);
  char strbuf[256];
  //Serial.printf("%s\n",to_string(sample,strbuf,256));

  // Empty / invalid sample from parser
  //if (sample.distance <= 0 || sample.angle < 0 || sample.angle >= 360) {
  //  return;
  //}

  //if(sample.start_flag) {
    //Serial.printf("New 360!\n");
    //Serial.printf("%s\n",to_string(sample,strbuf,256));
    
    //clearScanData();
  //  samplesThisFrame = 0;
  //}

  // Hi Arnav, I think this "write to screen" system needs to be optimized a bit
  // If you have any questions about the code feel free to text me
  addLidarPoint(sample.angle, sample.distance);
  samplesThisFrame++;
  drawScanPoints();

  if (samplesThisFrame > 5000) {
    clearScanData();
    drawScanHUD();
    samplesThisFrame = 0;
  }
#endif
#ifdef TEST_SERIAL
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

// Basic demo for accelerometer readings from Adafruit MPU6050

/*#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!

  // WHY THE FUCK DOES IT NOT INITIALIZE????
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {

  //Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Print out the values 
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}*/