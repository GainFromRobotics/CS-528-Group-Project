#ifndef OLED_HEADER
#define OLED_HEADER

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =======================
// OLED CONFIG
// =======================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_SDA 8
#define OLED_SCL 9

#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
extern Adafruit_SSD1306 display;

extern float distances[360];
extern bool validPoints[360];

extern unsigned long lastDrawMs;
#define DRAW_INTERVAL_MS 100

extern int samplesThisFrame;

// =======================
// FUNCTION DECLARATIONS
// =======================
void setupOLED();
void clearScanData();
void addLidarPoint(float angleDeg, float distanceMm);
void drawScanMap();
void drawStatusScreen(const char *line1, const char *line2);
#endif