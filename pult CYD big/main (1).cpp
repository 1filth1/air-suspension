#include <Arduino.h>
#include <LGFX_Sunton8048S043.h>

#define LCD_BL 2


// Forward declarations
void text(int x, int y, const char *s, uint16_t color, int size);
void drawValue(uint8_t n);
void drawStaticScreen();

LGFX_Sunton8048S043 display;
static const uint16_t BG = 0x0843;
static const uint16_t SURFACE = 0x21C9;
static const uint16_t MINT = 0xDFFF;
static const uint16_t SUB = 0xA5D7;
static const uint16_t ORANGE = 0xFD20;
float value[5] = {2.4f, 2.5f, 2.6f, 2.7f, 7.8f};
uint32_t lastUpdate = 0;

void text(int x, int y, const char *s, uint16_t color, int size) {
  display.setTextColor(color, SURFACE);
  display.setTextSize(size);
  display.setCursor(x, y);
  display.print(s);
}

void drawValue(uint8_t n) {
  int x = n == 4 ? 600 : ((n & 1) ? 420 : 30);
  int y = n == 4 ? 42 : ((n > 1) ? 292 : 122);
  int w = n == 4 ? 150 : 330;
  display.fillRect(x, y, w, 74, SURFACE);
  char b[20];
  snprintf(b, sizeof(b), "%.1f", value[n]);
  text(x, y + 7, b, MINT, n == 4 ? 3 : 5);
  text(x + (n == 4 ? 78 : 138), y + 43, "bar", SUB, 2);
}

void drawStaticScreen() {
  display.fillScreen(BG);
  display.setTextColor(MINT, BG);
  display.setTextSize(3);
  display.setCursor(25, 18);
  display.print("AIR RIDE  -  DISPLAY DRIVER TEST");
  display.setTextSize(1);
  display.setTextColor(SUB, BG);
  display.setCursor(25, 54);
  display.print("LovyanGFX native RGB scan-out / no Arduino_GFX flush");

  const char *name[4] = {"FL", "FR", "RL", "RR"};
  for (uint8_t i = 0; i < 4; ++i) {
    int x = (i & 1) ? 405 : 15;
    int y = (i > 1) ? 275 : 105;
    display.fillRoundRect(x, y, 380, 165, 16, SURFACE);
    display.drawRoundRect(x, y, 380, 165, 16, SUB);
    display.setTextColor(MINT, SURFACE);
    display.setTextSize(3);
    display.setCursor(x + 18, y + 16);
    display.print(name[i]);
    display.fillRoundRect(x + 155, y + 24, 190, 14, 7, 0x1166);
    display.fillRoundRect(x + 157, y + 26, 50 + i * 20, 10, 5, ORANGE);
  }
  display.fillRoundRect(585, 75, 200, 70, 12, SURFACE);
  display.setTextColor(MINT, SURFACE);
  display.setTextSize(2);
  display.setCursor(600, 84);
  display.print("TANK");
  drawValue(4);
  for (uint8_t i = 0; i < 4; ++i) drawValue(i);

  display.setTextColor(ORANGE, BG);
  display.setTextSize(2);
  display.setCursor(25, 455);
  display.print("TEST ONLY: this firmware sends no pneumatic commands");
}

void setup() {
  pinMode(LCD_BL, OUTPUT);
  analogWrite(LCD_BL, 255);
  display.begin();
  drawStaticScreen();
}

void loop() {
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    for (uint8_t i = 0; i < 5; ++i) {
      value[i] += (i & 1) ? 0.1f : -0.1f;
      if (value[i] < 1.0f) value[i] = 4.0f;
      if (value[i] > 9.5f) value[i] = 2.0f;
      drawValue(i);
    }
  }
  delay(5);
}