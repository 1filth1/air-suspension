#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <math.h>

// ST7789, SPI display in landscape orientation: 320 x 240.

struct PressurePacket {
  uint8_t marker;
  float bar[5];
  uint8_t flags;
} __attribute__((packed));

enum Button : uint8_t { UP, DOWN, MENU, NEXT, CONFIRM, PARK, DRIVE, LIFT, IN, OUT, PRESET, BUTTON_COUNT };

enum Page : uint8_t { HOME, SETTINGS, PICKER, EDITOR, CALIBRATION, BRIGHTNESS };

enum BleState : uint8_t { SEARCHING, CONNECTING, CONNECTED, RETRYING };


// Forward declarations
void printBar(float v);
void sendCommand(const uint8_t *p, size_t n);
void stopAll();
void tune(uint8_t w);
void valve(uint8_t w, bool fill);
void drawWheel(uint8_t i);
void drawTank();
void drawPreset();
void footer();
void home();
void header(const char *title);
void menuRow(uint8_t row, const char *text, bool selected);
void settings();
void picker();
void editor();
void calibration();
void render();
void choosePreset(uint8_t p);
void updateArrival();
void refreshTelemetry();
void onPressure(NimBLERemoteCharacteristic *, uint8_t *data, size_t n, bool);
void bluetooth();
void saveTarget();
void press(Button b);
void buttons();

static const uint8_t TFT_SCK = 18;
static const uint8_t TFT_MOSI = 23;
static const uint8_t TFT_CS = 4;
static const uint8_t TFT_DC = 16;

// Every switch has one leg on this GPIO and the other leg on GND.
// INPUT_PULLUP makes an unpressed button HIGH and a pressed button LOW.
static const uint8_t BTN_UP = 13;
static const uint8_t BTN_DOWN = 14;
static const uint8_t BTN_MENU = 19;
static const uint8_t BTN_NEXT = 21;
static const uint8_t BTN_OK = 22;
static const uint8_t BTN_PARK = 25;
static const uint8_t BTN_DRIVE = 26;
static const uint8_t BTN_LIFT = 27;
static const uint8_t BTN_IN = 32;
static const uint8_t BTN_OUT = 33;
static const uint8_t BTN_PRESET = 17;

static const char *SERVICE_UUID  = "a7b10000-7e5d-4a4c-9f09-8a0b01d40001";
static const char *PRESSURE_UUID = "a7b10001-7e5d-4a4c-9f09-8a0b01d40001";
static const char *COMMAND_UUID  = "a7b10002-7e5d-4a4c-9f09-8a0b01d40001";







static const uint16_t BG = 0x0843, CARD = 0x21C9, FG = 0xDFFF, DIM = 0xA5D7;
static const uint16_t ORANGE = 0xFD20, GREEN = 0x54AC, RED = 0xF800, LIME = 0xBFE0, BLUE = 0x05FF;

Adafruit_ST7789 tft(TFT_CS, TFT_DC, -1); // RESET is tied to the TFT board reset circuit.
Preferences prefs;
NimBLEClient *client = nullptr;
NimBLERemoteCharacteristic *commandChar = nullptr;
NimBLEAddress pendingAddress;

const char *wheel[4] = {"FL", "FR", "RL", "RR"};
const char *preset[3] = {"PARK", "DRIVE", "LIFT"};
const uint8_t buttonPins[BUTTON_COUNT] = {
  BTN_UP, BTN_DOWN, BTN_MENU, BTN_NEXT, BTN_OK, BTN_PARK, BTN_DRIVE, BTN_LIFT, BTN_IN, BTN_OUT, BTN_PRESET
};

float pressure[5] = {NAN, NAN, NAN, NAN, NAN};
uint16_t target[3][4] = {{20,20,20,20}, {55,55,55,55}, {80,80,80,80}};
int shownPressure[5] = {-9999, -9999, -9999, -9999, -9999};
bool lastButton[BUTTON_COUNT] = {};
bool telemetry = false;
bool linked = false, scanRequested = true, connectRequested = false, pendingValid = false;
bool manualMode = false, valveOpen = false, presetRunning = false, presetStopSent = false;
bool wheelComplete[4] = {}, wheelAlarm[4] = {};
float lockedPressure[4] = {NAN, NAN, NAN, NAN};
uint32_t completeAt[4] = {}, frameAt = 0, lastInput = 0, lastConnect = 0, lastStatus = 0;
bool blink = false;
uint8_t active = 0, cursor = 0, selectedWheel = 0, editPreset = 0, editWheel = 0, calSensor = 0;
uint16_t calValue = 0;
Page page = HOME;
BleState bleState = SEARCHING;

void printBar(float v) { if (isnan(v)) tft.print("--.-"); else tft.printf("%.1f", v); }
void sendCommand(const uint8_t *p, size_t n) {
  if (linked && client && client->isConnected() && commandChar && commandChar->canWrite()) commandChar->writeValue(p, n, true);
}
void stopAll() { const uint8_t p[] = {0xB1, 0x2F}; sendCommand(p, sizeof(p)); valveOpen = false; }
void tune(uint8_t w) {
  if (w > 3) return;
  const uint16_t v = target[active][w];
  const uint8_t p[] = {0xB1, 0x40, w, (uint8_t)v, (uint8_t)(v >> 8)};
  sendCommand(p, sizeof(p));
}
void valve(uint8_t w, bool fill) {
  if (w > 3) return;
  const uint8_t p[] = {0xB1, (uint8_t)(0x20 + w * 2 + (fill ? 0 : 1))};
  sendCommand(p, sizeof(p));
  valveOpen = true;
}

void drawWheel(uint8_t i) {
  const int x = (i & 1) ? 112 : 6;
  const int y = (i > 1) ? 112 : 38;
  const uint16_t border = (manualMode && i == selectedWheel) ? (blink ? BLUE : DIM) :
    ((wheelAlarm[i] && blink) ? RED : (wheelComplete[i] ? LIME : DIM));
  tft.fillRoundRect(x, y, 103, 66, 9, CARD);
  tft.drawRoundRect(x, y, 103, 66, 9, border);
  tft.drawRoundRect(x + 2, y + 2, 99, 62, 7, border);
  tft.setTextSize(1); tft.setTextColor(FG);
  tft.setCursor((i & 1) ? x + 9 : x + 77, y + 10); tft.print(wheel[i]);
  const bool leftGauge = (i & 1) == 0;
  const int gaugeX = leftGauge ? x + 8 : x + 81;
  const int numberX = leftGauge ? x + 29 : x + 11;
  tft.setFont(&FreeSansBold9pt7b); tft.setTextSize(2); tft.setTextColor(ORANGE);
  tft.setCursor(numberX, y + 51); printBar(pressure[i]); tft.setFont();
  const int level = isnan(pressure[i]) ? 0 : constrain((int)lroundf(pressure[i] * 10.0f), 0, 100);
  tft.fillRoundRect(gaugeX, y + 17, 14, 40, 4, 0x1166);
  const int h = level * 36 / 100;
  if (h) tft.fillRoundRect(gaugeX + 2, y + 55 - h, 10, h, 2, ORANGE);
}

void drawTank() {
  const int cx = 267, cy = 112;
  tft.fillRect(218, 31, 98, 158, BG);
  tft.drawCircle(cx, cy, 48, DIM); tft.drawCircle(cx, cy, 46, DIM);
  const int segments = isnan(pressure[4]) ? 0 : constrain((int)lroundf(pressure[4] * 28.0f / 15.0f), 0, 28);
  for (int i = 0; i < 28; ++i) {
    const float a = (142.0f + i * (256.0f / 28.0f)) * DEG_TO_RAD;
    const uint16_t c = i < segments ? ORANGE : 0x1166;
    tft.drawLine(cx + (int)(cosf(a) * 43), cy + (int)(sinf(a) * 43), cx + (int)(cosf(a) * 48), cy + (int)(sinf(a) * 48), c);
  }
  tft.setTextSize(1); tft.setTextColor(DIM); tft.setCursor(cx - 9, 76); tft.print("BAR");
  char text[10]; if (isnan(pressure[4])) strcpy(text, "--.-"); else snprintf(text, sizeof(text), "%.1f", pressure[4]);
  int16_t x1, y1; uint16_t w, h;
  tft.setFont(&FreeSansBold9pt7b); tft.setTextSize(2); tft.setTextColor(ORANGE);
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h); tft.setCursor(cx - w / 2 - x1, 120); tft.print(text); tft.setFont();
  tft.drawRoundRect(cx - 27, 145, 54, 15, 7, DIM); tft.setTextSize(1); tft.setTextColor(FG); tft.setCursor(cx - 12, 149); tft.print("TANK");
}

void drawPreset() {
  const uint16_t edge = presetRunning ? (blink ? RED : DIM) : (presetStopSent ? LIME : DIM);
  tft.fillRoundRect(225, 38, 89, 28, 10, CARD); tft.drawRoundRect(225, 38, 89, 28, 10, edge);
  const char *label = manualMode ? "MAN" : preset[active];
  int16_t x1, y1; uint16_t w, h;
  tft.setFont(&FreeSansBold9pt7b); tft.setTextSize(1); tft.setTextColor(LIME);
  tft.getTextBounds(label, 0, 0, &x1, &y1, &w, &h); tft.setCursor(269 - w / 2 - x1, 58); tft.print(label); tft.setFont();
}

void footer() {
  tft.fillRect(0, 202, 320, 38, BG); tft.drawFastHLine(5, 201, 310, DIM);
  const char *s = bleState == CONNECTED ? "CONNECTED" : bleState == CONNECTING ? "CONNECTING" : bleState == RETRYING ? "RETRYING" : "SEARCHING";
  tft.setTextSize(1); tft.setTextColor(bleState == CONNECTED ? GREEN : (bleState == CONNECTING ? ORANGE : RED)); tft.setCursor(8, 210); tft.print(s);
  tft.setTextColor(LIME); tft.setCursor(126, 210); tft.print(manualMode ? "MANUAL" : "AUTO");
  tft.setTextColor(FG); tft.setCursor(8, 228);
  if (manualMode) tft.print("UP/DOWN wheel | hold IN or OUT | MENU settings");
  else tft.print("PARK  DRIVE  LIFT | PRESET cycles | MENU settings");
}

void home() {
  tft.fillScreen(BG);
  for (uint8_t i = 0; i < 4; ++i) drawWheel(i);
  drawTank(); drawPreset(); footer();
  for (uint8_t i = 0; i < 5; ++i) shownPressure[i] = isnan(pressure[i]) ? -9999 : (int)lroundf(pressure[i] * 10.0f);
}
void header(const char *title) {
  tft.fillScreen(BG); tft.drawFastHLine(5, 27, 310, DIM);
  tft.setFont(&FreeSansBold9pt7b); tft.setTextSize(1); tft.setTextColor(FG); tft.setCursor(8, 20); tft.print(title); tft.setFont();
}
void menuRow(uint8_t row, const char *text, bool selected) {
  const int y = 42 + row * 29;
  tft.fillRoundRect(8, y, 304, 23, 7, selected ? ORANGE : BG);
  if (!selected) tft.drawRoundRect(8, y, 304, 23, 7, DIM);
  tft.setTextSize(1); tft.setTextColor(selected ? BG : FG); tft.setCursor(19, y + 8); tft.print(text);
}
void settings() { header("MENU"); const char *m[] = {"START PRESET", "EDIT PRESETS", "CALIBRATE", "BACK TO DASHBOARD"}; for (uint8_t i = 0; i < 4; ++i) menuRow(i, m[i], i == cursor); }
void picker() { header("START PRESET"); for (uint8_t i = 0; i < 3; ++i) menuRow(i, preset[i], i == cursor); tft.setTextColor(DIM); tft.setCursor(10, 155); tft.print("OK sends the selected targets"); }
void editor() { header("EDIT PRESETS"); tft.setTextColor(DIM); tft.setCursor(12, 42); tft.print(preset[editPreset]); for (uint8_t i = 0; i < 4; ++i) { const int y = 55 + i * 31; const bool sel = i == editWheel; tft.fillRoundRect(8, y, 304, 24, 7, sel ? ORANGE : BG); if (!sel) tft.drawRoundRect(8, y, 304, 24, 7, DIM); tft.setTextColor(sel ? BG : FG); tft.setCursor(18, y + 8); tft.print(wheel[i]); tft.setCursor(248, y + 8); tft.printf("%.1f", target[editPreset][i] / 10.0f); } }
void calibration() { header("CALIBRATE"); for (uint8_t i = 0; i < 5; ++i) { const int y = 40 + i * 27; const bool sel = i == calSensor; tft.fillRoundRect(8, y, 304, 21, 7, sel ? ORANGE : BG); if (!sel) tft.drawRoundRect(8, y, 304, 21, 7, DIM); tft.setTextColor(sel ? BG : FG); tft.setCursor(18, y + 7); tft.print(i < 4 ? wheel[i] : "TANK"); tft.setCursor(245, y + 7); if (sel) tft.printf("%.1f", calValue / 10.0f); else printBar(pressure[i]); } }
void render() { if (page == HOME) home(); else if (page == SETTINGS) settings(); else if (page == PICKER) picker(); else if (page == EDITOR) editor(); else calibration(); }

void choosePreset(uint8_t p) {
  active = p % 3; prefs.putUChar("preset", active); manualMode = false; presetRunning = true; presetStopSent = false; blink = true; frameAt = millis();
  for (uint8_t i = 0; i < 4; ++i) { wheelComplete[i] = false; wheelAlarm[i] = false; completeAt[i] = 0; lockedPressure[i] = NAN; tune(i); }
  if (page == HOME) home();
}
void updateArrival() {
  const uint32_t now = millis();
  if (now - frameAt >= 250U) { frameAt = now; blink = !blink; if (page == HOME && (presetRunning || manualMode)) { for (uint8_t i = 0; i < 4; ++i) drawWheel(i); drawPreset(); } }
  if (!presetRunning) return;
  bool all = true;
  for (uint8_t i = 0; i < 4; ++i) {
    const float goal = target[active][i] / 10.0f;
    const bool at = !isnan(pressure[i]) && fabsf(pressure[i] - goal) <= 0.2f;
    if (!wheelComplete[i] && at) { if (!completeAt[i]) completeAt[i] = now; else if (now - completeAt[i] >= 2000U) { wheelComplete[i] = true; lockedPressure[i] = pressure[i]; } }
    else if (!wheelComplete[i]) completeAt[i] = 0;
    if (wheelComplete[i] && !isnan(lockedPressure[i]) && !isnan(pressure[i]) && pressure[i] < lockedPressure[i] - 0.5f) wheelAlarm[i] = true;
    if (wheelAlarm[i] && at) wheelAlarm[i] = false;
    if (!wheelComplete[i]) all = false;
  }
  if (all && !presetStopSent) { stopAll(); presetStopSent = true; presetRunning = false; }
}
void refreshTelemetry() { for (uint8_t i = 0; i < 5; ++i) { const int n = isnan(pressure[i]) ? -9999 : (int)lroundf(pressure[i] * 10.0f); if (n != shownPressure[i]) { shownPressure[i] = n; if (page == HOME) { if (i < 4) drawWheel(i); else drawTank(); } } } }

void onPressure(NimBLERemoteCharacteristic *, uint8_t *data, size_t n, bool) { if (n == sizeof(PressurePacket) && data[0] == 0xA1) { PressurePacket p; memcpy(&p, data, sizeof(p)); for (uint8_t i = 0; i < 5; ++i) pressure[i] = p.bar[i]; telemetry = true; } }
class ClientCB : public NimBLEClientCallbacks { void onDisconnect(NimBLEClient *, int) override { linked = false; commandChar = nullptr; scanRequested = true; bleState = RETRYING; } } clientCB;
class ScanCB : public NimBLEScanCallbacks { void onResult(const NimBLEAdvertisedDevice *d) override { if (!linked && !connectRequested && d->isAdvertisingService(NimBLEUUID(SERVICE_UUID))) { pendingAddress = d->getAddress(); pendingValid = true; connectRequested = true; bleState = CONNECTING; NimBLEDevice::getScan()->stop(); } } } scanCB;
void bluetooth() {
  NimBLEScan *scan = NimBLEDevice::getScan();
  if (connectRequested && pendingValid && !scan->isScanning()) {
    connectRequested = false; pendingValid = false; lastConnect = millis();
    if (client) NimBLEDevice::deleteClient(client);
    client = NimBLEDevice::createClient(); client->setClientCallbacks(&clientCB, false); client->setConnectTimeout(10);
    if (client->connect(pendingAddress)) { NimBLERemoteService *svc = client->getService(SERVICE_UUID); commandChar = svc ? svc->getCharacteristic(COMMAND_UUID) : nullptr; NimBLERemoteCharacteristic *pc = svc ? svc->getCharacteristic(PRESSURE_UUID) : nullptr; if (commandChar && commandChar->canWrite() && pc && pc->canNotify() && pc->subscribe(true, onPressure)) { linked = true; bleState = CONNECTED; if (page == HOME) footer(); return; } client->disconnect(); }
    scanRequested = true; bleState = RETRYING;
  }
  if (!linked && scanRequested && !scan->isScanning() && millis() - lastConnect > 800U) { scanRequested = false; bleState = SEARCHING; scan->setScanCallbacks(&scanCB, false); scan->setActiveScan(true); scan->setInterval(80); scan->setWindow(40); scan->start(0, false); }
}

void saveTarget() { char k[8]; snprintf(k, sizeof(k), "p%uw%u", editPreset, editWheel); prefs.putUShort(k, target[editPreset][editWheel]); }
void press(Button b) {
  lastInput = millis();
  if (page == HOME) {
    if (b == PARK) choosePreset(0); else if (b == DRIVE) choosePreset(1); else if (b == LIFT) choosePreset(2); else if (b == PRESET) choosePreset(active + 1);
    else if (b == MENU) { stopAll(); page = SETTINGS; cursor = 0; render(); }
    else if (b == CONFIRM) { manualMode = !manualMode; stopAll(); home(); }
    else if (manualMode && b == UP) { selectedWheel = (selectedWheel + 3) % 4; home(); }
    else if (manualMode && (b == DOWN || b == NEXT)) { selectedWheel = (selectedWheel + 1) % 4; home(); }
    else if (manualMode && b == IN) valve(selectedWheel, true);
    else if (manualMode && b == OUT) valve(selectedWheel, false);
    return;
  }
  if (page == SETTINGS) { if (b == UP && cursor) cursor--; else if (b == DOWN && cursor < 3) cursor++; else if (b == CONFIRM) { if (cursor == 0) { page = PICKER; cursor = 0; } else if (cursor == 1) { page = EDITOR; editPreset = active; editWheel = 0; } else if (cursor == 2) { page = CALIBRATION; calSensor = 0; calValue = 0; } else page = HOME; } else if (b == MENU) page = HOME; render(); return; }
  if (page == PICKER) { if (b == UP && cursor) cursor--; else if (b == DOWN && cursor < 2) cursor++; else if (b == CONFIRM) { choosePreset(cursor); page = HOME; } else if (b == MENU) page = SETTINGS; render(); return; }
  if (page == EDITOR) { if (b == UP && target[editPreset][editWheel] < 150) target[editPreset][editWheel]++; else if (b == DOWN && target[editPreset][editWheel]) target[editPreset][editWheel]--; else if (b == CONFIRM) editWheel = (editWheel + 1) % 4; else if (b == NEXT) editPreset = (editPreset + 1) % 3; else if (b == MENU) { page = SETTINGS; cursor = 1; } saveTarget(); render(); return; }
  if (page == CALIBRATION) { if (b == UP && calValue < 150) calValue++; else if (b == DOWN && calValue) calValue--; else if (b == NEXT) calSensor = (calSensor + 1) % 5; else if (b == CONFIRM) { const uint8_t p[] = {0xB1, 0x41, calSensor, (uint8_t)calValue, (uint8_t)(calValue >> 8)}; sendCommand(p, sizeof(p)); } else if (b == MENU) { page = SETTINGS; cursor = 2; } render(); }
}
void buttons() {
  // Handle releases first. This guarantees that switching from OUT to IN
  // closes the old valve before the new hold command is sent.
  bool downNow[BUTTON_COUNT];
  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
    downNow[i] = digitalRead(buttonPins[i]) == LOW;
    if ((i == IN || i == OUT) && lastButton[i] && !downNow[i] && valveOpen) {
      stopAll();
      if (page == HOME) footer();
    }
  }
  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) {
    if (downNow[i] && !lastButton[i]) press((Button)i);
    lastButton[i] = downNow[i];
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < BUTTON_COUNT; ++i) { pinMode(buttonPins[i], INPUT_PULLUP); lastButton[i] = digitalRead(buttonPins[i]) == LOW; }
  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS); tft.init(240, 320); tft.setRotation(1);
  prefs.begin("airride", false); active = prefs.getUChar("preset", 0); if (active > 2) active = 0;
  for (uint8_t a = 0; a < 3; ++a) for (uint8_t b = 0; b < 4; ++b) { char k[8]; snprintf(k, sizeof(k), "p%uw%u", a, b); target[a][b] = prefs.getUShort(k, target[a][b]); }
  NimBLEDevice::init("AirRide Remote"); NimBLEDevice::setMTU(64);
  home();
}
void loop() {
  bluetooth(); buttons();
  if (telemetry) { telemetry = false; refreshTelemetry(); }
  updateArrival();
  if (page == HOME && millis() - lastStatus > 1500U) { lastStatus = millis(); footer(); }
  if (page != HOME && millis() - lastInput > 60000U) { page = HOME; home(); }
  delay(15);
}