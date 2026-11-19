// Air Suspension — keychain remote for the TTGO T-Display (ESP32, 1.14" IPS).
//
// Board: TTGO T-Display, ESP32, ST7789V 135x240 over 4-wire SPI.
// Display shows the tank pressure and all four wheel pressures; four buttons
// raise or lower the front and the rear axle.
//
// The remote speaks only the part of the controller protocol that is identical
// in the v2 (main) and v3 (release candidate) controller configs, so it works
// with either firmware:
//   E1 01 / E1 81 / E1 02   discovery, encrypted-peer handshake, heartbeat
//   A1 + float[5] + flags   telemetry, 250 ms
//   B1 20..27               manual pulse for one wheel: fill/dump
//   B1 30..33               cancel the manual pulse on one wheel
//   B1 2F                   stop every valve and the compressor
// The settings frames (0xA2 / 0xA3 / 0x43 / 0x44 / 0x45) are deliberately not
// used: they differ between v2 and v3, and this remote does not need them.
//
// Safety model, inherited from the controller:
//   - A manual pulse expires in the controller after manualTimeoutMs (2 s by
//     default), so a lost link closes the valves instead of leaving them open.
//     That is why every held button re-sends its command every 180 ms.
//   - The controller still enforces the wheel and tank limits; the checks here
//     only avoid sending obviously pointless commands.
//   - After the link is re-established a button has to be released once before
//     it can open a valve again, so a remote that was held during a dropout
//     does not start pumping by itself.

#include <Adafruit_GFX.h>
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* Board pins — TTGO T-Display (ESP32)                                       */
/* ------------------------------------------------------------------------- */

#define LCD_SCLK 18
#define LCD_MOSI 19
#define LCD_CS 5
#define LCD_DC 16
#define LCD_RST 23
#define LCD_BL 4

// Built-in active-low buttons. GPIO35 uses the onboard pull-up.
#define BTN_SELECT 0
#define BTN_CONFIRM 35

// The battery divider is powered from a GPIO to keep the drain at zero.
// Set KEYCHAIN_BATTERY 0 if the revision you have has no divider.
#ifndef KEYCHAIN_BATTERY
#define KEYCHAIN_BATTERY 1
#endif
#define BATTERY_ADC 34
#define BATTERY_EN 14

/* ------------------------------------------------------------------------- */
/* ST7789 over SPI — minimal Adafruit_GFX driver                             */
/* ------------------------------------------------------------------------- */

class ST7789Spi : public Adafruit_GFX {
public:
  static const int16_t PANEL_W = 135;
  static const int16_t PANEL_H = 240;

  ST7789Spi() : Adafruit_GFX(PANEL_W, PANEL_H) {}

  void begin() {
    pinMode(LCD_CS, OUTPUT);
    digitalWrite(LCD_CS, HIGH);
    pinMode(LCD_DC, OUTPUT);
    digitalWrite(LCD_DC, HIGH);
    pinMode(LCD_RST, OUTPUT);
    digitalWrite(LCD_RST, HIGH);

    spi.begin(LCD_SCLK, -1, LCD_MOSI, LCD_CS);
    spi.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));

    digitalWrite(LCD_RST, HIGH); delay(10);
    digitalWrite(LCD_RST, LOW);  delay(20);
    digitalWrite(LCD_RST, HIGH); delay(150);

    cmd(0x11);  // sleep out
    delay(120);
    cmd(0x36); dat(0x00);  // MADCTL: portrait, RGB order
    cmd(0x3A); dat(0x55);  // 16 bit colour
    cmd(0xB2); dat(0x0C); dat(0x0C); dat(0x00); dat(0x33); dat(0x33);
    cmd(0xB7); dat(0x35);
    cmd(0xBB); dat(0x19);
    cmd(0xC0); dat(0x2C);
    cmd(0xC2); dat(0x01);
    cmd(0xC3); dat(0x12);
    cmd(0xC4); dat(0x20);
    cmd(0xC6); dat(0x0F);
    cmd(0xD0); dat(0xA4); dat(0xA1);
    cmd(0xE0);
    dat(0xD0); dat(0x04); dat(0x0D); dat(0x11); dat(0x13); dat(0x2B);
    dat(0x3F); dat(0x54); dat(0x4C); dat(0x18); dat(0x0D); dat(0x0B);
    dat(0x1F); dat(0x23);
    cmd(0xE1);
    dat(0xD0); dat(0x04); dat(0x0C); dat(0x11); dat(0x13); dat(0x2C);
    dat(0x3F); dat(0x44); dat(0x51); dat(0x2F); dat(0x1F); dat(0x1F);
    dat(0x20); dat(0x23);
    cmd(0x21);  // inversion on — the T-Display panel needs it
    cmd(0x29);  // display on
    delay(20);
  }

  void fillScreen(uint16_t color) override {
    fillRect(0, 0, _width, _height, color);
  }

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x < 0 || y < 0 || x >= _width || y >= _height) return;
    window(x, y, x, y);
    cmd(0x2C);
    dcHigh();
    digitalWrite(LCD_CS, LOW);  // Keep the panel selected while sending pixels.
    spi.write16(color);
    csHigh();
  }

  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
    if (w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > _width) w = _width - x;
    if (y + h > _height) h = _height - y;
    if (w <= 0 || h <= 0) return;

    window(x, y, x + w - 1, y + h - 1);
    cmd(0x2C);
    dcHigh();
    digitalWrite(LCD_CS, LOW);  // Keep the panel selected while sending pixels.

    // 32 pixels per transfer keeps the stack small and the bus busy.
    uint8_t block[64];
    for (uint8_t i = 0; i < 32; i++) {
      block[i * 2] = color >> 8;
      block[i * 2 + 1] = color & 0xFF;
    }
    uint32_t pixels = (uint32_t)w * h;
    while (pixels > 0) {
      const uint16_t chunk = pixels > 32 ? 32 : (uint16_t)pixels;
      spi.writeBytes(block, chunk * 2);
      pixels -= chunk;
    }
    csHigh();
  }

  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override {
    fillRect(x, y, w, 1, color);
  }

  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override {
    fillRect(x, y, 1, h, color);
  }

private:
  // A 135x240 panel sits inside a 240x320 controller memory, so every window
  // has to be shifted by the panel origin.
  static const uint16_t COL_OFFSET = 52;
  static const uint16_t ROW_OFFSET = 40;

  void dcLow() { digitalWrite(LCD_DC, LOW); }
  void dcHigh() { digitalWrite(LCD_DC, HIGH); }
  void csHigh() { digitalWrite(LCD_CS, HIGH); }

  void cmd(uint8_t value) {
    dcLow();
    digitalWrite(LCD_CS, LOW);
    spi.write(value);
    csHigh();
    delayMicroseconds(1);
  }

  void dat(uint8_t value) {
    dcHigh();
    digitalWrite(LCD_CS, LOW);
    spi.write(value);
    csHigh();
  }

  void window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    const uint16_t xs = x0 + COL_OFFSET;
    const uint16_t xe = x1 + COL_OFFSET;
    const uint16_t ys = y0 + ROW_OFFSET;
    const uint16_t ye = y1 + ROW_OFFSET;
    cmd(0x2A);
    dat(xs >> 8); dat(xs & 0xFF); dat(xe >> 8); dat(xe & 0xFF);
    cmd(0x2B);
    dat(ys >> 8); dat(ys & 0xFF); dat(ye >> 8); dat(ye & 0xFF);
  }

  SPIClass spi;
};

static ST7789Spi gfx;

/* ------------------------------------------------------------------------- */
/* Palette and layout                                                        */
/* ------------------------------------------------------------------------- */

static const uint16_t BG = 0x0843, SURFACE = 0x21C9, MINT = 0xDFFF, SUB = 0xA5D7,
                      ORANGE = 0xFD20, GREEN = 0x54AC, RED = 0xF800,
                      GAUGE_BG = 0x1166;

namespace layout {
// Header.
const int16_t HEADER_Y = 5;
const int16_t LINK_X = 128, LINK_Y = 9, LINK_R = 4;

// Tank card.
const int16_t TANK_X = 3, TANK_Y = 21, TANK_W = 129, TANK_H = 59;
const int16_t TANK_LABEL_X = 9, TANK_LABEL_Y = 25;
const int16_t TANK_VALUE_X = 9, TANK_VALUE_Y = 40;
const int16_t TANK_GAUGE_X = 9, TANK_GAUGE_Y = 71, TANK_GAUGE_W = 117, TANK_GAUGE_H = 4;

// Wheel cards: 2 columns, 2 rows.
const int16_t WHEEL_X[2] = {3, 69};
const int16_t WHEEL_Y[2] = {84, 151};
const int16_t WHEEL_W = 63, WHEEL_H = 64;
const int16_t WHEEL_LABEL_DX = 7, WHEEL_LABEL_DY = 8;
const int16_t WHEEL_VALUE_DX = 7, WHEEL_VALUE_DY = 23;
const int16_t WHEEL_GAUGE_DX = 7, WHEEL_GAUGE_DY = 53;
const int16_t WHEEL_GAUGE_W = 49, WHEEL_GAUGE_H = 5;

}  // namespace layout

/* ------------------------------------------------------------------------- */
/* Link state and telemetry                                                  */
/* ------------------------------------------------------------------------- */

struct Telemetry {
  uint8_t marker;
  float bar[5];
  uint8_t flags;
} __attribute__((packed));
static_assert(sizeof(Telemetry) == 22, "telemetry layout changed");

struct SettingsFrame {
  uint8_t marker, version;
  uint32_t generation;
  uint16_t wheelLimitDbar[4];
  uint16_t presetDbar[3][4];
  uint16_t tankLimitDbar, compressorOnDbar, targetToleranceDbar;
  uint16_t autoTimeoutSec, compressorMaxRunSec, manualTimeoutMs;
  uint8_t calibratedMask;
  uint16_t calibrationReferenceDbar[5];
  int16_t zeroMilliVolts[5];
  char presetNames[3][13];
  uint32_t check;
} __attribute__((packed));
static_assert(sizeof(SettingsFrame) == 114, "settings frame layout changed");

static uint32_t checkBytes(const uint8_t *bytes, size_t length) {
  uint32_t value = 0x6D2B79F5UL;
  for (size_t i = 0; i < length; ++i) {
    value = (value << 5) | (value >> 27);
    value ^= bytes[i];
  }
  return value;
}

static const uint8_t ESPNOW_CHANNEL = 1;
static const uint8_t ESPNOW_BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// Same key as the controller and the other remotes. See README: this belongs in
// per-device storage, not in a public repository.
static const uint8_t ESPNOW_LMK[16] = {0x41, 0x69, 0x72, 0x52, 0x69, 0x64,
                                       0x65, 0x2D, 0x32, 0x30, 0x32, 0x36,
                                       0x2D, 0x4B, 0x31, 0x21};
static const uint32_t CONTROLLER_TIMEOUT_MS = 1200;
static const uint32_t DISCOVERY_INTERVAL_MS = 500;
// The controller switches to its encrypted peer 150 ms after it queued the
// clear discovery reply, so the encrypted confirmation has to arrive later.
static const uint32_t CONFIRM_DELAY_MS = 250;
static const uint32_t TELEMETRY_STALE_MS = 1200;

static bool espNowReady = false;
static bool controllerPeerKnown = false;
static bool linked = false;
static uint8_t controllerMac[6];
static uint32_t lastControllerPacketMs = 0;
static uint32_t lastDiscoveryMs = 0;
static uint32_t discoveryConfirmAtMs = 0;
static bool discoveryConfirmPending = false;

static Telemetry receivedTelemetry;
static volatile bool telemetryPending = false;
static SettingsFrame receivedSettings = {};
static volatile bool settingsPending = false;
static bool controllerSettingsLoaded = false;
static char presetNames[3][13] = {"PARK", "DRIVE", "LIFT"};
static uint16_t presetTargets[3][4] = {{20,20,20,20},{55,55,55,55},{80,80,80,80}};
static uint32_t settingsGeneration = 0;
static uint32_t lastSettingsRequestMs = 0;
static uint32_t lastTelemetryMs = 0;

// bar[0..3] are the wheels FL, FR, RL, RR; bar[4] is the tank.
static float pressureBar[5] = {NAN, NAN, NAN, NAN, NAN};
static uint8_t telemFlags = 0;

static const uint8_t FLAG_SENSORS_OK = 1;
static const uint8_t FLAG_COMPRESSOR = 2;
static const uint8_t FLAG_LOCKOUT = 4;

static bool telemetryFresh() {
  return linked && (millis() - lastTelemetryMs) < TELEMETRY_STALE_MS;
}

static bool barValid(float bar) {
  return !isnan(bar) && bar >= -0.2f && bar <= 25.0f;
}

static float wheelBar(uint8_t wheel) {
  return telemetryFresh() ? pressureBar[wheel] : NAN;
}

static float tankBar() {
  return telemetryFresh() ? pressureBar[4] : NAN;
}

static bool tankValid() {
  return barValid(tankBar());
}

/* ------------------------------------------------------------------------- */
/* Sending                                                                   */
/* ------------------------------------------------------------------------- */

static void sendCommand(const uint8_t *payload, size_t length) {
  if (!espNowReady || !controllerPeerKnown || !linked || length == 0 ||
      length > 250) {
    return;
  }
  if (esp_now_send(controllerMac, payload, length) != ESP_OK) {
    linked = false;
  }
}

static void sendManual(uint8_t wheel, bool fill) {
  // Controller: wheel = (opcode - 0x20) / 2, fill = !(opcode & 1).
  const uint8_t p[] = {0xB1, (uint8_t)(0x20 + wheel * 2 + (fill ? 0 : 1))};
  sendCommand(p, sizeof(p));
}

static void sendCancel(uint8_t wheel) {
  const uint8_t p[] = {0xB1, (uint8_t)(0x30 + wheel)};
  sendCommand(p, sizeof(p));
}

static void sendDiscovery() {
  const uint8_t hello[2] = {0xE1, 0x01};
  if (espNowReady) esp_now_send(ESPNOW_BROADCAST, hello, sizeof(hello));
}

/* ------------------------------------------------------------------------- */
/* ESP-NOW                                                                   */
/* ------------------------------------------------------------------------- */

static bool addControllerPeer(const uint8_t *mac) {
  if (controllerPeerKnown && memcmp(controllerMac, mac, 6) == 0) return true;
  if (controllerPeerKnown) esp_now_del_peer(controllerMac);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;
  memcpy(peer.lmk, ESPNOW_LMK, sizeof(ESPNOW_LMK));
  if (esp_now_add_peer(&peer) != ESP_OK) {
    // Never keep a peer record the radio does not have: every later send would
    // fail and the link would stay down until the next reboot.
    controllerPeerKnown = false;
    memset(controllerMac, 0, sizeof(controllerMac));
    return false;
  }
  memcpy(controllerMac, mac, 6);
  controllerPeerKnown = true;
  return true;
}

static void handleEspNowFrame(const uint8_t *mac, const uint8_t *data, int len) {
  if (mac == nullptr || data == nullptr) return;

  // The controller answers discovery in the clear, then switches this MAC to
  // the shared encrypted key. The confirmation is sent from loop() later.
  if (len == 2 && data[0] == 0xE1 && data[1] == 0x81) {
    if (addControllerPeer(mac)) {
      discoveryConfirmAtMs = millis() + CONFIRM_DELAY_MS;
      discoveryConfirmPending = true;
    }
    return;
  }

  if (!controllerPeerKnown || memcmp(controllerMac, mac, 6) != 0) return;
  if (len == (int)sizeof(SettingsFrame) && data[0] == 0xA2) {
    SettingsFrame frame;
    memcpy(&frame, data, sizeof(frame));
    if (frame.version == 2 &&
        frame.check == checkBytes((const uint8_t *)&frame, offsetof(SettingsFrame, check))) {
      memcpy(&receivedSettings, &frame, sizeof(frame));
      settingsPending = true;
      lastControllerPacketMs = millis();
    }
    return;
  }
  if (len != (int)sizeof(Telemetry) || data[0] != 0xA1) return;
  memcpy(&receivedTelemetry, data, sizeof(receivedTelemetry));
  lastControllerPacketMs = millis();
  telemetryPending = true;
}

// Arduino-ESP32 3.x hands the receive callback an esp_now_recv_info_t and the
// send callback a wifi_tx_info_t; 2.x passes the peer MAC directly. The project
// builds the other remotes on 2.0.17, while a fresh Arduino IDE installs 3.x,
// so this remote has to compile on both.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data,
                            int len) {
  handleEspNowFrame(info != nullptr ? info->src_addr : nullptr, data, len);
}

static void onEspNowSend(const wifi_tx_info_t *, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS && linked) linked = false;
}
#else
static void onEspNowReceive(const uint8_t *mac, const uint8_t *data, int len) {
  handleEspNowFrame(mac, data, len);
}

static void onEspNowSend(const uint8_t *, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS && linked) linked = false;
}
#endif

static void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK ||
      esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_set_pmk(ESPNOW_LMK);
  esp_now_register_recv_cb(onEspNowReceive);
  esp_now_register_send_cb(onEspNowSend);

  esp_now_peer_info_t broadcastPeer = {};
  memcpy(broadcastPeer.peer_addr, ESPNOW_BROADCAST, 6);
  broadcastPeer.channel = ESPNOW_CHANNEL;
  broadcastPeer.ifidx = WIFI_IF_STA;
  broadcastPeer.encrypt = false;
  if (esp_now_add_peer(&broadcastPeer) != ESP_OK) {
    Serial.println("ESP-NOW broadcast peer failed");
    return;
  }
  espNowReady = true;
}

/* ------------------------------------------------------------------------- */
/* Buttons                                                                   */
/* ------------------------------------------------------------------------- */

static const uint8_t kButtonPins[2] = {BTN_SELECT, BTN_CONFIRM};
static const uint32_t DEBOUNCE_MS = 25;
static const uint32_t REPEAT_INTERVAL_MS = 180;
static bool buttonRaw[2] = {};
static bool buttonStable[2] = {};
static uint32_t buttonRawAt[2] = {};
static int8_t selection = -1; // Manual wheel/action: FL fill; eight states.
static int8_t activeSelection = -1;
static bool waitForRelease = true;
static uint32_t sentAt = 0;
static bool manualMode = false;
static uint8_t selectedPreset = 0;
static uint32_t selectPressedAt = 0;
static uint32_t lastButtonActivityAt = 0;
static const uint32_t MANUAL_HOLD_MS = 2000;
static const uint32_t MANUAL_IDLE_MS = 2000;
static const uint8_t PRESET_COMMAND = 0x44;

static void sendPreset(uint8_t preset) {
  if (preset >= 3 || !linked || !controllerSettingsLoaded) return;
  const uint8_t command[] = {0xB1, PRESET_COMMAND, preset};
  sendCommand(command, sizeof(command));
}

static void pollButtons(uint32_t now) {
  for (uint8_t i = 0; i < 2; ++i) {
    bool raw = digitalRead(kButtonPins[i]) == LOW;
    if (raw != buttonRaw[i]) {
      buttonRaw[i] = raw;
      buttonRawAt[i] = now;
    } else if (raw != buttonStable[i] && now - buttonRawAt[i] >= DEBOUNCE_MS) {
      buttonStable[i] = raw;
    }
  }
}

static void invalidateAxes() {
  activeSelection = -1;
  waitForRelease = true;
}

static void handleButtons(uint32_t now) {
  static bool selectWasPressed = false;
  static bool confirmWasPressed = false;
  const bool selectPressed = buttonStable[0];
  const bool confirmPressed = buttonStable[1];
  const bool selectPressedEdge = selectPressed && !selectWasPressed;
  const bool selectReleasedEdge = !selectPressed && selectWasPressed;
  const bool confirmPressedEdge = confirmPressed && !confirmWasPressed;
  const bool confirmReleasedEdge = !confirmPressed && confirmWasPressed;
  selectWasPressed = selectPressed;
  confirmWasPressed = confirmPressed;
  if (selectPressedEdge) selectPressedAt = now;

  // A held button is activity: never time out during a manual command.
  if (buttonRaw[0] || buttonRaw[1] || buttonStable[0] || buttonStable[1])
    lastButtonActivityAt = now;

  // Holding SELECT enters manual mode; a short click selects the next preset.
  if (!manualMode && selectPressed && buttonRaw[0] &&
      now - selectPressedAt >= MANUAL_HOLD_MS) {
    manualMode = true;
    selection = 0; // Start with front-left fill.
    lastButtonActivityAt = now;
    waitForRelease = true; // CONFIRM must be released after entry.
  } else if (manualMode && selectPressedEdge) {
    if (activeSelection >= 0) sendCancel(activeSelection / 2);
    activeSelection = -1;
    selection = (selection + 1) % 8;
    if (buttonStable[1] || buttonRaw[1]) waitForRelease = true;
  } else if (!manualMode && selectReleasedEdge &&
             now - selectPressedAt < MANUAL_HOLD_MS &&
             linked && controllerSettingsLoaded) {
    selectedPreset = (selectedPreset + 1) % 3;
  }

  // Outside manual mode, a short CONFIRM click applies the highlighted preset.
  if (!manualMode && confirmReleasedEdge &&
      linked && controllerSettingsLoaded) {
    sendPreset(selectedPreset);
  }

  if (manualMode && now - lastButtonActivityAt >= MANUAL_IDLE_MS) {
    if (activeSelection >= 0) sendCancel(activeSelection / 2);
    manualMode = false;
    selection = -1;
    invalidateAxes();
  }
  if (!manualMode) return;
  if (!linked || !telemetryFresh()) {
    if (activeSelection >= 0) sendCancel(activeSelection / 2);
    invalidateAxes();
    return;
  }
  if (waitForRelease) {
    if (!buttonRaw[1] && !buttonStable[1]) waitForRelease = false;
    return;
  }
  // LOCK is a compressor lockout, not a manual-valve prohibition.
  // SENS is aggregate: a fault on another wheel must not disable this one.
  // Mirror the controller manualStart checks; its pressure limits remain authoritative.
  bool allowed = selection >= 0 && buttonStable[1] && buttonRaw[1] &&
                 barValid(wheelBar(selection / 2)) &&
                 ((selection & 1) || tankValid());
  if (!allowed) {
    if (activeSelection >= 0) sendCancel(activeSelection / 2);
    activeSelection = -1;
    return;
  }
  if (activeSelection != selection || now - sentAt >= REPEAT_INTERVAL_MS) {
    sendManual(selection / 2, !(selection & 1));
    activeSelection = linked ? selection : -1;
    sentAt = now;
    if (!linked) waitForRelease = true;
  }
}

/* ------------------------------------------------------------------------- */
/* Rendering                                                                 */
/* ------------------------------------------------------------------------- */

static char shownTank[8] = "";
static char shownWheel[4][8] = {"", "", "", ""};
static char shownBattery[6] = "";
static char shownStatus[6] = "";
static int8_t shownLink = -1;

static void drawText(int16_t x, int16_t y, const char *text, uint16_t fg,
                     uint16_t bg, uint8_t size) {
  gfx.setTextSize(size);
  gfx.setTextColor(fg, bg);
  gfx.setCursor(x, y);
  gfx.print(text);
}

static void drawTextRight(int16_t right, int16_t y, const char *text,
                          uint16_t fg, uint16_t bg, uint8_t size) {
  const int16_t width = (int16_t)strlen(text) * 6 * size;
  drawText(right - width, y, text, fg, bg, size);
}

static void formatBar(char *out, size_t length, float bar) {
  if (!barValid(bar)) {
    snprintf(out, length, "--.-");
    return;
  }
  snprintf(out, length, "%.1f", bar);
}

static void drawGauge(int16_t x, int16_t y, int16_t w, int16_t h, float fraction,
                      uint16_t color) {
  gfx.fillRect(x, y, w, h, GAUGE_BG);
  if (!(fraction > 0.0f)) return;
  if (fraction > 1.0f) fraction = 1.0f;
  int16_t filled = (int16_t)lroundf(fraction * w);
  if (filled < 1) filled = 1;
  gfx.fillRect(x, y, filled, h, color);
}

static void drawLinkState(bool force) {
  int8_t state = 0;  // 0 radio off, 1 searching, 2 linked
  if (espNowReady) state = linked ? 2 : 1;
  if (!force && state == shownLink) return;
  shownLink = state;
  gfx.fillCircle(layout::LINK_X, layout::LINK_Y, layout::LINK_R,
                 state == 2 ? GREEN : (state == 1 ? ORANGE : RED));
}

#if KEYCHAIN_BATTERY
static uint8_t batteryPercent() {
  digitalWrite(BATTERY_EN, HIGH);
  delay(2);
  const int raw = analogRead(BATTERY_ADC);
  digitalWrite(BATTERY_EN, LOW);
  // 1:2 divider, 4.2 V full / 3.3 V empty cell.
  const float volts = (float)raw * 3.3f / 4095.0f * 2.0f;
  float percent = (volts - 3.30f) / (4.20f - 3.30f) * 100.0f;
  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;
  return (uint8_t)percent;
}

static void drawBattery(uint8_t percent, bool force) {
  char text[6];
  snprintf(text, sizeof(text), "%u%%", (unsigned)percent);
  if (!force && strcmp(text, shownBattery) == 0) return;
  gfx.fillRect(88, layout::HEADER_Y, 30, 8, BG);
  snprintf(shownBattery, sizeof(shownBattery), "%s", text);
  const uint16_t color = percent < 20 ? RED : (percent < 40 ? ORANGE : GREEN);
  drawTextRight(118, layout::HEADER_Y, text, color, BG, 1);
}
#endif

static void applyControllerSettings() {
  if (!settingsPending) return;
  SettingsFrame frame;
  noInterrupts();
  memcpy(&frame, &receivedSettings, sizeof(frame));
  settingsPending = false;
  interrupts();
  if (frame.marker != 0xA2 || frame.version != 2 ||
      frame.check != checkBytes((const uint8_t *)&frame, offsetof(SettingsFrame, check))) return;
  for (uint8_t p = 0; p < 3; ++p) {
    if (frame.presetNames[p][12] != '\0' || strnlen(frame.presetNames[p], 12) == 0) return;
  }
  memcpy(presetNames, frame.presetNames, sizeof(presetNames));
  memcpy(presetTargets, frame.presetDbar, sizeof(presetTargets));
  settingsGeneration = frame.generation;
  controllerSettingsLoaded = true;
}

static void drawPresetFooter(bool force) {
  static int8_t shownPreset = -1;
  static bool shownManual = false;
  static bool shownLoaded = false;
  const bool changed = force || shownManual != manualMode ||
                       shownLoaded != controllerSettingsLoaded ||
                       shownPreset != (int8_t)selectedPreset;
  static char shownText[32] = "";
  char text[32] = "";
  if (manualMode) {
    snprintf(text, sizeof(text), "MANUAL");
  } else if (!controllerSettingsLoaded) {
    snprintf(text, sizeof(text), "%s", linked ? "SYNC SETTINGS..." : "NO CONTROLLER");
  } else {
    const uint8_t p = selectedPreset;
    snprintf(text, sizeof(text), ">%.6s %.1f/%.1f/%.1f/%.1f",
      presetNames[p],
      presetTargets[p][0] / 10.0f, presetTargets[p][1] / 10.0f,
      presetTargets[p][2] / 10.0f, presetTargets[p][3] / 10.0f);
  }
  if (changed || strcmp(text, shownText) != 0) {
    gfx.fillRect(0, 218, 135, 22, BG);
    drawText(2, 222, text, controllerSettingsLoaded ? MINT : ORANGE, BG, 1);
    snprintf(shownText, sizeof(shownText), "%s", text);
  }
  shownPreset = (int8_t)selectedPreset;
  shownManual = manualMode;
  shownLoaded = controllerSettingsLoaded;
}

static void drawStatusWord(bool force) {
  const char *word = "";
  uint16_t color = SUB;
  if (telemFlags & FLAG_LOCKOUT) {
    word = "LOCK";
    color = RED;
  } else if (!(telemFlags & FLAG_SENSORS_OK)) {
    word = "SENS";
    color = RED;
  } else if (telemFlags & FLAG_COMPRESSOR) {
    word = "PUMP";
    color = ORANGE;
  }
  if (!force && strcmp(word, shownStatus) == 0) return;
  gfx.fillRect(96, layout::TANK_LABEL_Y - 1, 30, 9, SURFACE);
  snprintf(shownStatus, sizeof(shownStatus), "%s", word);
  if (word[0] != '\0') {
    drawTextRight(126, layout::TANK_LABEL_Y, word, color, SURFACE, 1);
  }
}

static void drawTankValue(bool force) {
  char text[8];
  formatBar(text, sizeof(text), tankBar());
  if (!force && strcmp(text, shownTank) == 0) return;
  snprintf(shownTank, sizeof(shownTank), "%s", text);
  gfx.fillRect(layout::TANK_VALUE_X, layout::TANK_VALUE_Y, 4 * 18, 24, SURFACE);
  drawText(layout::TANK_VALUE_X, layout::TANK_VALUE_Y, text, MINT, SURFACE, 3);
}

static void drawTankGauge(bool force) {
  static int16_t shownFilled = -1;
  const float fraction = tankBar() / 16.0f;
  const int16_t filled =
      barValid(tankBar()) ? (int16_t)lroundf((fraction > 1.0f ? 1.0f : fraction) *
                                             layout::TANK_GAUGE_W)
                          : 0;
  if (!force && filled == shownFilled) return;
  shownFilled = filled;
  drawGauge(layout::TANK_GAUGE_X, layout::TANK_GAUGE_Y, layout::TANK_GAUGE_W,
            layout::TANK_GAUGE_H, fraction, ORANGE);
}

static void drawWheelValue(uint8_t wheel, bool force) {
  char text[8];
  formatBar(text, sizeof(text), wheelBar(wheel));
  if (!force && strcmp(text, shownWheel[wheel]) == 0) return;
  snprintf(shownWheel[wheel], sizeof(shownWheel[wheel]), "%s", text);
  const int16_t x = layout::WHEEL_X[wheel & 1] + layout::WHEEL_VALUE_DX;
  const int16_t y = layout::WHEEL_Y[wheel >> 1] + layout::WHEEL_VALUE_DY;
  gfx.fillRect(x, y, 4 * 12, 16, SURFACE);
  const float bar = wheelBar(wheel);
  uint16_t color = MINT;
  if (!barValid(bar)) {
    color = SUB;
  } else if (bar > 9.5f) {
    color = ORANGE;  // close to the usual 10 bar wheel limit
  }
  drawText(x, y, text, color, SURFACE, 2);
}

static void drawWheelGauge(uint8_t wheel, bool force) {
  static int16_t shownFilled[4] = {-1, -1, -1, -1};
  const float bar = wheelBar(wheel);
  const float fraction = bar / 10.0f;
  const int16_t filled =
      barValid(bar) ? (int16_t)lroundf((fraction > 1.0f ? 1.0f : fraction) *
                                       layout::WHEEL_GAUGE_W)
                    : 0;
  if (!force && filled == shownFilled[wheel]) return;
  shownFilled[wheel] = filled;
  drawGauge(layout::WHEEL_X[wheel & 1] + layout::WHEEL_GAUGE_DX,
            layout::WHEEL_Y[wheel >> 1] + layout::WHEEL_GAUGE_DY,
            layout::WHEEL_GAUGE_W, layout::WHEEL_GAUGE_H, fraction, MINT);
}

static void drawStaticFrame() {
  gfx.fillScreen(BG);

  drawText(4, layout::HEADER_Y, "AIR RIDE", MINT, BG, 1);

  // Tank card.
  gfx.fillRoundRect(layout::TANK_X, layout::TANK_Y, layout::TANK_W,
                    layout::TANK_H, 8, SURFACE);
  drawText(layout::TANK_LABEL_X, layout::TANK_LABEL_Y, "TANK", SUB, SURFACE, 1);
  drawText(84, layout::TANK_VALUE_Y + 14, "bar", SUB, SURFACE, 1);

  // Wheel cards.
  static const char *names[4] = {"FL", "FR", "RL", "RR"};
  for (uint8_t wheel = 0; wheel < 4; wheel++) {
    const int16_t x = layout::WHEEL_X[wheel & 1];
    const int16_t y = layout::WHEEL_Y[wheel >> 1];
    gfx.fillRoundRect(x, y, layout::WHEEL_W, layout::WHEEL_H, 8, SURFACE);
    drawText(x + layout::WHEEL_LABEL_DX, y + layout::WHEEL_LABEL_DY, names[wheel],
             SUB, SURFACE, 1);
  }

}

static void drawSelection(bool force) {
  static int8_t shown = -2;
  static bool shownOn = false;
  bool on = activeSelection < 0 || ((millis() / 250) % 2 == 0);
  if (!force && shown == selection && shownOn == on) return;
  for (uint8_t wheel = 0; wheel < 4; ++wheel) {
    uint16_t color = SURFACE;
    if (selection >= 0 && wheel == selection / 2 && on)
      color = (selection & 1) ? RED : 0x001F;
    for (int16_t inset = 0; inset < 2; ++inset)
      gfx.drawRoundRect(layout::WHEEL_X[wheel & 1] + inset,
                        layout::WHEEL_Y[wheel >> 1] + inset,
                        layout::WHEEL_W - inset * 2,
                        layout::WHEEL_H - inset * 2, 8 - inset, color);
  }
  shown = selection;
  shownOn = on;
}

static void render(bool full) {
  if (full) {
    shownTank[0] = '\0';
    for (uint8_t i = 0; i < 4; i++) shownWheel[i][0] = '\0';
    shownStatus[0] = '\0';
    shownBattery[0] = '\0';
    shownLink = -1;
    drawStaticFrame();
  }
  drawLinkState(full);
  drawStatusWord(full);
  drawTankValue(full);
  drawTankGauge(full);
  for (uint8_t wheel = 0; wheel < 4; wheel++) {
    drawWheelValue(wheel, full);
    drawWheelGauge(wheel, full);
  }
  drawSelection(full);
  drawPresetFooter(full);
}

/* ------------------------------------------------------------------------- */
/* Setup / loop                                                              */
/* ------------------------------------------------------------------------- */

void setup() {
  Serial.begin(115200);

  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_CONFIRM, INPUT);
#if KEYCHAIN_BATTERY
  pinMode(BATTERY_EN, OUTPUT);
  digitalWrite(BATTERY_EN, LOW);
  pinMode(BATTERY_ADC, INPUT);
  analogSetPinAttenuation(BATTERY_ADC, ADC_11db);
#endif

  pinMode(LCD_BL, OUTPUT);
  analogWrite(LCD_BL, 255);

  gfx.begin();
  render(true);

  setupEspNow();
  // Only the status dot and the placeholders change here, so the frame is not
  // repainted: a second full-screen redraw would flash at boot.
  render(false);

  // Nothing is sent until a controller answers discovery, so a fresh remote
  // never opens a valve on its own.
  waitForRelease = true;
}

void loop() {
  const uint32_t now = millis();

  pollButtons(now);

  // Discovery, handshake and link watchdog.
  if (discoveryConfirmPending && (int32_t)(now - discoveryConfirmAtMs) >= 0) {
    discoveryConfirmPending = false;
    if (controllerPeerKnown) {
      const uint8_t confirm[2] = {0xE1, 0x02};
      esp_now_send(controllerMac, confirm, sizeof(confirm));
      lastDiscoveryMs = now;
    }
  }

  const bool fresh = controllerPeerKnown &&
                     lastControllerPacketMs != 0 &&
                     (now - lastControllerPacketMs) < CONTROLLER_TIMEOUT_MS;
  if (fresh != linked) {
    linked = fresh;
    if (!fresh) {
      // A rebooted controller has no encrypted peer, so drop ours as well and
      // let the next clear E1 81 rebuild the whole handshake.
      if (controllerPeerKnown) {
        esp_now_del_peer(controllerMac);
        controllerPeerKnown = false;
        memset(controllerMac, 0, sizeof(controllerMac));
      }
      lastControllerPacketMs = 0;
      controllerSettingsLoaded = false;
      lastSettingsRequestMs = 0;
      lastDiscoveryMs = now;
      sendDiscovery();
      invalidateAxes();
    }
  }

  if (!linked && now - lastDiscoveryMs >= DISCOVERY_INTERVAL_MS) {
    lastDiscoveryMs = now;
    sendDiscovery();
  } else if (linked && now - lastDiscoveryMs >= DISCOVERY_INTERVAL_MS) {
    lastDiscoveryMs = now;
    const uint8_t heartbeat[2] = {0xE1, 0x02};
    esp_now_send(controllerMac, heartbeat, sizeof(heartbeat));
  }

  applyControllerSettings();

  if (linked && !controllerSettingsLoaded &&
      (lastSettingsRequestMs == 0 || now - lastSettingsRequestMs >= 750U)) {
    const uint8_t request[] = {0xB1, 0x42};
    sendCommand(request, sizeof(request));
    lastSettingsRequestMs = now;
  }

  if (telemetryPending) {
    noInterrupts();
    Telemetry packet = receivedTelemetry;
    telemetryPending = false;
    interrupts();
    for (uint8_t i = 0; i < 5; i++) pressureBar[i] = packet.bar[i];
    telemFlags = packet.flags;
    lastTelemetryMs = now;
  }

  handleButtons(now);

#if KEYCHAIN_BATTERY
  static uint32_t lastBatteryMs = 0;
  static uint8_t battery = 100;
  if (now - lastBatteryMs >= 5000 || lastBatteryMs == 0) {
    lastBatteryMs = now;
    battery = batteryPercent();
    drawBattery(battery, false);
  }
#endif

  render(false);
  delay(2);
}