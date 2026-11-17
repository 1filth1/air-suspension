#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

// Built-in hardware on ESP32-2432S028R.

// Forward declarations
int16_t screenX(int16_t rawX);
int16_t screenY(int16_t rawY);
void drawLabel(int16_t x, int16_t y, const char *text, uint16_t color, uint8_t size);
void drawFixedScreen();
void drawReading(bool touched, int16_t rawX, int16_t rawY);

constexpr uint8_t TFT_BL_PIN = 21;
constexpr uint8_t TOUCH_CS_PIN = 33;
constexpr uint8_t TOUCH_IRQ_PIN = 36;
constexpr uint8_t TOUCH_SCK_PIN = 25;
constexpr uint8_t TOUCH_MISO_PIN = 39;
constexpr uint8_t TOUCH_MOSI_PIN = 32;

// Initial calibration estimates. The raw readings shown on screen are used
// to replace these with values measured from this specific panel.
constexpr int16_t RAW_X_LEFT = 200;
constexpr int16_t RAW_X_RIGHT = 3700;
constexpr int16_t RAW_Y_TOP = 240;
constexpr int16_t RAW_Y_BOTTOM = 3800;

TFT_eSPI tft = TFT_eSPI();
SPIClass touchSPI(VSPI);
XPT2046_Touchscreen touch(TOUCH_CS_PIN, TOUCH_IRQ_PIN);

bool lastTouched = false;
int16_t lastRawX = -1;
int16_t lastRawY = -1;
uint32_t lastUpdateMs = 0;

int16_t screenX(int16_t rawX) {
  return constrain(map(rawX, RAW_X_LEFT, RAW_X_RIGHT, 0, 239), 0, 239);
}

int16_t screenY(int16_t rawY) {
  return constrain(map(rawY, RAW_Y_TOP, RAW_Y_BOTTOM, 0, 319), 0, 319);
}

void drawLabel(int16_t x, int16_t y, const char *text, uint16_t color, uint8_t size) {
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextSize(size);
  tft.setCursor(x, y);
  tft.print(text);
}

void drawFixedScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.fillRect(0, 0, 240, 35, TFT_NAVY);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("TOUCH CHECK");

  drawLabel(12, 52, "Touch the screen", TFT_LIGHTGREY, 2);
  drawLabel(12, 78, "This test has no BLE.", TFT_DARKGREY, 1);
  tft.drawRoundRect(8, 105, 224, 92, 7, TFT_DARKGREY);
  tft.drawRoundRect(8, 211, 224, 101, 7, TFT_DARKGREY);
}

void drawReading(bool touched, int16_t rawX, int16_t rawY) {
  // Redraw only the two data boxes. The rest of the display stays unchanged.
  tft.fillRoundRect(10, 107, 220, 88, 6, TFT_BLACK);
  drawLabel(18, 120, touched ? "TOUCH DETECTED" : "NO TOUCH", touched ? TFT_GREEN : TFT_RED, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(18, 154);
  if (touched) {
    tft.printf("raw %d, %d", rawX, rawY);
  } else {
    tft.print("raw - , -");
  }

  tft.fillRoundRect(10, 213, 220, 97, 6, TFT_BLACK);
  drawLabel(18, 225, "SCREEN POSITION", TFT_CYAN, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(18, 263);
  if (touched) {
    tft.printf("x %d   y %d", screenX(rawX), screenY(rawY));
  } else {
    tft.print("x -    y -");
  }
  drawLabel(18, 293, "Send raw values from 3 points.", TFT_DARKGREY, 1);
}

void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, HIGH);
  tft.init();
  tft.setRotation(0);  // Portrait: 240 x 320.
  drawFixedScreen();

  // XPT2046 is wired to the board's separate VSPI bus. Begin the SPI bus
  // first, then call touch.begin() without arguments (the XPT2046_Touchscreen
  // library version present does not accept an SPIClass argument).
  touchSPI.begin(TOUCH_SCK_PIN, TOUCH_MISO_PIN, TOUCH_MOSI_PIN, TOUCH_CS_PIN);
  touch.begin();
  touch.setRotation(0);

  drawReading(false, -1, -1);
  Serial.println("Touch diagnostic started");
}

void loop() {
  const bool touched = touch.touched();
  int16_t rawX = -1;
  int16_t rawY = -1;

  if (touched) {
    const TS_Point point = touch.getPoint();
    rawX = point.x;
    rawY = point.y;
  }

  const bool changed = (touched != lastTouched) ||
                       (touched && (abs(rawX - lastRawX) > 8 || abs(rawY - lastRawY) > 8));
  if (changed && (millis() - lastUpdateMs >= 50 || touched != lastTouched)) {
    drawReading(touched, rawX, rawY);
    lastUpdateMs = millis();
    if (touched) {
      Serial.printf("raw %d, %d  screen %d, %d\n", rawX, rawY, screenX(rawX), screenY(rawY));
    } else {
      Serial.println("no touch");
    }
  }

  lastTouched = touched;
  lastRawX = rawX;
  lastRawY = rawY;
  delay(5);
}