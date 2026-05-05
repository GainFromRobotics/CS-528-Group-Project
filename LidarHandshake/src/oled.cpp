#include "oled.h"

#include <math.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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

int samplesThisFrame = 0;

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