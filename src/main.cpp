#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#include "rplidarc1.h"

// =======================
// OLED CONFIG
// =======================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_SDA 8
#define OLED_SCL 9

#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =======================
// LIDAR CONFIG
// =======================
// LiDAR TX -> ESP32 RX
// LiDAR RX -> ESP32 TX
#define RPLIDAR_RX 4
#define RPLIDAR_TX 5

RPLidarC1 lidar(2, RPLIDAR_RX, RPLIDAR_TX, 460800);
response_descriptor scanDescriptor;
bool scanStarted = false;

// =======================
// SCAN MAP CONFIG
// =======================
const int CENTER_X = SCREEN_WIDTH / 2;
const int CENTER_Y = SCREEN_HEIGHT / 2;
const int MAP_RADIUS_PIXELS = 30;

const float MAX_DISTANCE_MM = 12000.0;  // 12 meters

float distances[360];
bool validPoints[360];

unsigned long lastDrawMs = 0;
const unsigned long DRAW_INTERVAL_MS = 100;

int samplesThisFrame = 0;

// =======================
// FUNCTION DECLARATIONS
// =======================
void setupOLED();
void setupLidar();
void clearScanData();
void addLidarPoint(float angleDeg, float distanceMm);
void drawScanMap();
void drawStatusScreen(const char *line1, const char *line2);

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupOLED();
  clearScanData();

  drawStatusScreen("2D Scan Map", "Starting LiDAR");
  setupLidar();
}

void loop() {
  if (!scanStarted) {
    drawStatusScreen("LiDAR error", "Scan not started");
    delay(1000);
    return;
  }

  scan_standard_sample sample = lidar.get_latest_scan(scanDescriptor);

  // Empty / invalid sample from parser
  if (sample.distance <= 0 || sample.angle < 0 || sample.angle >= 360) {
    return;
  }


  // start_flag means the LiDAR has started a new 360-degree rotation.
  // So draw the previous rotation, then clear for the next one.
  if (sample.start_flag && samplesThisFrame > 20) {
    drawScanMap();
    clearScanData();
    samplesThisFrame = 0;
  }

  addLidarPoint(sample.angle, sample.distance);
  samplesThisFrame++;

  // Backup draw timer in case start_flag parsing is imperfect.
  if (millis() - lastDrawMs > DRAW_INTERVAL_MS) {
    drawScanMap();
    lastDrawMs = millis();
  }
}

// =======================
// OLED SETUP
// =======================
void setupOLED() {
  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 init failed");
    while (true) {
      delay(100);
    }
  }

  display.clearDisplay();
  display.display();

  Serial.println("OLED initialized");
}

// =======================
// LIDAR SETUP
// =======================
void setupLidar() {
  lidar.begin();

  delay(100);

  // Stop any previous mode, then start standard scan mode.
  lidar.cmd_stop();
  delay(20);

  scanDescriptor = lidar.cmd_scan();

  Serial.println("Scan descriptor:");
  Serial.print("start_flag1=");
  Serial.print(scanDescriptor.start_flag1, HEX);
  Serial.print(" start_flag2=");
  Serial.print(scanDescriptor.start_flag2, HEX);
  Serial.print(" data_len=");
  Serial.print(scanDescriptor.data_len);
  Serial.print(" send_mode=");
  Serial.print(scanDescriptor.send_mode);
  Serial.print(" data_type=");
  Serial.println(scanDescriptor.data_type, HEX);

  if (scanDescriptor.data_len != SCAN_STANDARD_PACKET_LENGTH) {
    scanStarted = false;
    Serial.println("Failed to start standard scan.");
    drawStatusScreen("LiDAR error", "Bad descriptor");
    return;
  }

  scanStarted = true;
  Serial.println("LiDAR standard scan started.");

  drawStatusScreen("LiDAR OK", "Scanning...");
  delay(500);
}

// =======================
// CLEAR OLD FRAME
// =======================
void clearScanData() {
  for (int i = 0; i < 360; i++) {
    distances[i] = 0;
    validPoints[i] = false;
  }
}

// =======================
// ADD ONE LIDAR POINT
// =======================
void addLidarPoint(float angleDeg, float distanceMm) {
  if (distanceMm <= 0 || distanceMm > MAX_DISTANCE_MM) {
    return;
  }

  int angleIndex = round(angleDeg);

  while (angleIndex < 0) {
    angleIndex += 360;
  }

  while (angleIndex >= 360) {
    angleIndex -= 360;
  }

  // If multiple samples land in the same degree bucket,
  // keep the closer one because it usually represents the obstacle boundary.
  if (!validPoints[angleIndex] || distanceMm < distances[angleIndex]) {
    distances[angleIndex] = distanceMm;
    validPoints[angleIndex] = true;
  }
}

// =======================
// DRAW SCAN MAP
// =======================
void drawScanMap() {
  display.clearDisplay();

  // Border
  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);


  // Center point: LiDAR position
  display.fillCircle(CENTER_X, CENTER_Y, 1, SSD1306_WHITE);

  // Direction marker: front/up
  display.drawLine(CENTER_X, CENTER_Y, CENTER_X, CENTER_Y - 6, SSD1306_WHITE);

  for (int angle = 0; angle < 360; angle++) {
    if (!validPoints[angle]) {
      continue;
    }

    float distanceMm = distances[angle];
    float radiusPixels = (distanceMm / MAX_DISTANCE_MM) * MAP_RADIUS_PIXELS;

    // 0 degrees = forward/up
    // 90 degrees = right
    // 180 degrees = back/down
    // 270 degrees = left
    float angleRad = angle * PI / 180.0;

    int x = CENTER_X + round(sin(angleRad) * radiusPixels);
    int y = CENTER_Y - round(cos(angleRad) * radiusPixels);

    if (x >= 1 && x < SCREEN_WIDTH - 1 && y >= 1 && y < SCREEN_HEIGHT - 1) {
      display.drawPixel(x, y, SSD1306_WHITE);
    }
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(2, 2);
  display.print("Scan");

  display.setCursor(2, 54);
  display.print("Max ");
  display.print((int)(MAX_DISTANCE_MM / 1000));
  display.print("m");

  display.display();
}

// =======================
// STATUS SCREEN
// =======================
void drawStatusScreen(const char *line1, const char *line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(line1);

  display.setCursor(0, 12);
  display.println(line2);

  display.display();
}