#include <Arduino.h>
#include <esp32-hal-cpu.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <math.h>
#include <stddef.h>
#include <soc/gpio_reg.h>
#include <soc/soc.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#define LCD_POWER 15
#define LCD_BL 38
#define LCD_RST 5
#define LCD_CS 6
#define LCD_DC 7
#define LCD_WR 8
#define LCD_RD 9
#define LCD_D0 39
#define LCD_D1 40
#define LCD_D2 41
#define LCD_D3 42
#define LCD_D4 45
#define LCD_D5 46
#define LCD_D6 47
#define LCD_D7 48
// Home screen: MENU, mode, three presets, and re-run selected preset.
#define BTN_MENU 10
#define BTN_MODE 12
#define BTN_DRIVE 1
#define BTN_PARK 11
#define BTN_LIFT 2
#define BTN_FORCE_EXHAUST 3
// Manual mode: each button is held only while its valve must remain open.
#define BTN_FL_IN 17
#define BTN_FR_IN 13
#define BTN_FL_OUT 21
#define BTN_FR_OUT 16
#define BTN_RL_IN 11
#define BTN_RR_IN 1
#define BTN_RL_OUT 3
#define BTN_RR_OUT 2
// Menu navigation uses the same physical four buttons.
#define BTN_UP 1
#define BTN_DOWN 3
#define BTN_OK 2
#define BTN_BACK 11
#define BATTERY_ADC 4
#define BUZZER_PIN 18
#define PRESET_LED_PIN 44
#define MANUAL_LED_PIN 43
// ESP-NOW uses Wi-Fi channel 1 and the same 16-byte key as the controller.

struct PressurePacket { uint8_t marker; float bar[5]; uint8_t flags; } __attribute__((packed));

struct EspNowDiscoveryPacket { uint8_t marker; uint8_t type; } __attribute__((packed));
struct SettingsFrame {
  uint8_t marker,version; uint32_t generation;
  uint16_t wheelLimitDbar[4]; uint16_t presetDbar[3][4];
  uint16_t tankLimitDbar; uint16_t compressorOnDbar; uint16_t targetToleranceDbar;
  uint16_t autoTimeoutSec; uint16_t compressorMaxRunSec; uint16_t manualTimeoutMs;
  uint8_t calibratedMask; uint16_t calibrationReferenceDbar[5]; int16_t zeroMilliVolts[5];
  uint32_t check;
} __attribute__((packed));
static_assert(sizeof(SettingsFrame) == 75, "Settings frame layout changed");
uint32_t checkBytes(const uint8_t *bytes, size_t length) {
  uint32_t value = 0x6D2B79F5UL;
  for (size_t i = 0; i < length; ++i) { value = (value << 5) | (value >> 27); value ^= bytes[i]; }
  return value;
}

enum Key : uint8_t { UP = 1, DOWN = 2, BACK = 3, NEXT = 4, KEYOK = 5,
                     PARK_KEY = 6, DRIVE_KEY = 7, LIFT_KEY = 8,
                     IN_KEY = 9, OUT_KEY = 10, PRESET_KEY = 11, NONE = 255 };

enum BleState { BLE_SEARCHING, BLE_CONNECTING, BLE_CONNECTED, BLE_RETRYING };


// Forward declarations

// Forward declarations
void updateBatteryChargeAnimation();

void onEspNowReceive(const uint8_t *mac_addr, const uint8_t *data, int len);
void onEspNowSend(const const uint8_t *, esp_now_send_status_t status);

constexpr uint8_t ESPNOW_CHANNEL = 1;
const uint8_t ESPNOW_LMK[16] = {0x41,0x69,0x72,0x52,0x69,0x64,0x65,0x2D,0x32,0x30,0x32,0x36,0x2D,0x4B,0x31,0x21};
const uint8_t ESPNOW_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
constexpr uint32_t CONTROLLER_TIMEOUT_MS = 1200UL;
constexpr uint32_t DISCOVERY_INTERVAL_MS = 500UL;

// Sentinel values used by the manual-valve action handler. These are not
// physical GPIO pins; they are only compared with heldValveButton to track
// which virtual button is currently being held.
#define BTN_IN  100
#define BTN_OUT 101

#define TFT_WIDTH  170
#define TFT_HEIGHT 320



// Command packets retain the existing B1 command format so the controller's
// valve and calibration logic can be reused unchanged. E1 01 is discovery;
// E1 02 is a heartbeat and never controls a valve.

static_assert(sizeof(PressurePacket) == 22, "Pressure packet layout changed");






// Forward declarations

// Forward declarations

// Forward declarations



// Forward declarations

// Forward declarations
void ledBright();
void volume();

void updatePresetLightAnimation();

void enterControllerLostSleep();
bool anyWakeButtonPressed();

void drawEditorHeader();
void drawEditorWheel(uint8_t index, bool selected);
void drawCalibrationSensor(uint8_t index, bool selected);

void menuLine(uint8_t n, const char *s, bool selected);
int batteryPercent();
bool usbPowerConnected();
void drawBottomStatus(bool force);
void drawTankSegment(uint8_t segment, uint16_t color);
int tankSegmentCount();
void updateWheelReading(uint8_t index, int x, int y);
void updatePresetLabel();
void drawPresetFrame();
void drawWheelArrivalState(uint8_t index, int x, int y);
void updatePresetArrival();
void wheelCard(uint8_t index, int x, int y);
void drawTankGauge();
void updateTankGauge();
void drawBleStatus();
void updateHomeTelemetry();
void updateManualWheelFrame();
void startBeep(uint16_t frequencyHz, uint16_t durationMs);
void updateBuzzer();
void redrawWheelFrame(uint8_t index);
void updateButtonLights(bool force = false);
uint8_t presetLedDuty();
uint8_t buzzerDuty();

const uint16_t BG=0x0843, FG=0xDFFF, DIM=0xA5D7, ORANGE=0xFD20, TANK_ORANGE=0xFE00, GREEN=0x54AC, RED=0xF800, LIME=0xBFE0, BLUE=0x05FF;

class ST7789_Parallel : public Adafruit_GFX {
public:
  ST7789_Parallel(int16_t w, int16_t h) : Adafruit_GFX(w, h) {}

  void begin() {
    const uint8_t dataPins[8] = {LCD_D0,LCD_D1,LCD_D2,LCD_D3,LCD_D4,LCD_D5,LCD_D6,LCD_D7};
    for(int i=0;i<8;i++){
      pinMode(dataPins[i], OUTPUT);
      digitalWrite(dataPins[i], LOW);
    }
    pinMode(LCD_CS, OUTPUT); digitalWrite(LCD_CS, HIGH);
    pinMode(LCD_DC, OUTPUT); digitalWrite(LCD_DC, HIGH);
    pinMode(LCD_WR, OUTPUT); digitalWrite(LCD_WR, HIGH);
    pinMode(LCD_RD, OUTPUT); digitalWrite(LCD_RD, HIGH);
    pinMode(LCD_RST, OUTPUT);

    digitalWrite(LCD_RST, HIGH); delay(10);
    digitalWrite(LCD_RST, LOW);  delay(20);
    digitalWrite(LCD_RST, HIGH); delay(150);

    writeCommand(0x11); delay(120);
    writeCommand(0x36); writeData(0x00);
    writeCommand(0x3A); writeData(0x55);
    writeCommand(0xB2); writeData(0x0C); writeData(0x0C); writeData(0x00); writeData(0x33); writeData(0x33);
    writeCommand(0xB7); writeData(0x35);
    writeCommand(0xBB); writeData(0x19);
    writeCommand(0xC0); writeData(0x2C);
    writeCommand(0xC2); writeData(0x01);
    writeCommand(0xC3); writeData(0x12);
    writeCommand(0xC4); writeData(0x20);
    writeCommand(0xC6); writeData(0x0F);
    writeCommand(0xD0); writeData(0xA4); writeData(0xA1);
    writeCommand(0xE0);
    writeData(0xD0); writeData(0x04); writeData(0x0D); writeData(0x11);
    writeData(0x13); writeData(0x2B); writeData(0x3F); writeData(0x54);
    writeData(0x4C); writeData(0x18); writeData(0x0D); writeData(0x0B);
    writeData(0x1F); writeData(0x23);
    writeCommand(0xE1);
    writeData(0xD0); writeData(0x04); writeData(0x0C); writeData(0x11);
    writeData(0x13); writeData(0x2C); writeData(0x3F); writeData(0x44);
    writeData(0x51); writeData(0x2F); writeData(0x1F); writeData(0x1F);
    writeData(0x20); writeData(0x23);
    writeCommand(0x21);
    writeCommand(0x29); delay(10);
    setAddrWindow(0, 0, TFT_WIDTH-1, TFT_HEIGHT-1);
  }

  void setRotation(uint8_t r) override {
    rotation = r & 3;
    Adafruit_GFX::setRotation(rotation);
    uint8_t madctl = 0;
    switch(rotation) {
      case 0: madctl = 0x00; _width=TFT_WIDTH;  _height=TFT_HEIGHT; break;
      case 1: madctl = 0x60; _width=TFT_HEIGHT; _height=TFT_WIDTH;  break;
      case 2: madctl = 0xC0; _width=TFT_WIDTH;  _height=TFT_HEIGHT; break;
      case 3: madctl = 0xA0; _width=TFT_HEIGHT; _height=TFT_WIDTH;  break;
    }
    writeCommand(0x36); writeData(madctl);
  }

  void fillScreen(uint16_t color) {
    setAddrWindow(0, 0, _width-1, _height-1);
    writeCommand(0x2C);
    uint8_t hi = color >> 8, lo = color & 0xFF;
    digitalWrite(LCD_DC, HIGH);
    digitalWrite(LCD_CS, LOW);
    uint32_t total = (uint32_t)_width * _height;
    for(uint32_t i=0; i<total; i++){
      writeByte(hi);
      writeByte(lo);
    }
    digitalWrite(LCD_CS, HIGH);
  }

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if(x<0||y<0||x>=_width||y>=_height) return;
    setAddrWindow(x, y, x, y);
    writeCommand(0x2C);
    digitalWrite(LCD_DC, HIGH);
    digitalWrite(LCD_CS, LOW);
    writeByte(color>>8);
    writeByte(color&0xFF);
    digitalWrite(LCD_CS, HIGH);
  }

  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
    if(w<=0||h<=0) return;
    if(x<0){w+=x;x=0;}
    if(y<0){h+=y;y=0;}
    if(x+w>_width)  w=_width-x;
    if(y+h>_height) h=_height-y;
    if(w<=0||h<=0) return;
    setAddrWindow(x, y, x+w-1, y+h-1);
    writeCommand(0x2C);
    uint8_t hi=color>>8, lo=color&0xFF;
    digitalWrite(LCD_DC, HIGH);
    digitalWrite(LCD_CS, LOW);
    uint32_t total=(uint32_t)w*h;
    for(uint32_t i=0;i<total;i++){writeByte(hi);writeByte(lo);}
    digitalWrite(LCD_CS, HIGH);
  }

private:
  inline void writeByte(uint8_t b) {
    // D0..D3 are GPIO39..42 and D4..D7 are GPIO45..48.  They all
    // live in ESP32-S3's upper GPIO register bank, so write all eight
    // data lines at once instead of calling digitalWrite() eight times.
    constexpr uint32_t dataMask = 0x0FU << 7 | 0xF0U << 9;
    const uint32_t dataBits = ((uint32_t)(b & 0x0F) << 7) |
                              ((uint32_t)(b & 0xF0) << 9);
    REG_WRITE(GPIO_OUT1_W1TC_REG, dataMask);
    REG_WRITE(GPIO_OUT1_W1TS_REG, dataBits);
    // WR is GPIO8 in the lower register bank. One direct pulse latches
    // the byte into the ST7789 much faster than two digitalWrite calls.
    REG_WRITE(GPIO_OUT_W1TC_REG, 1UL << LCD_WR);
    REG_WRITE(GPIO_OUT_W1TS_REG, 1UL << LCD_WR);
  }

  void writeCommand(uint8_t cmd) {
    digitalWrite(LCD_DC, LOW);
    digitalWrite(LCD_CS, LOW);
    writeByte(cmd);
    digitalWrite(LCD_CS, HIGH);
  }

  void writeData(uint8_t d) {
    digitalWrite(LCD_DC, HIGH);
    digitalWrite(LCD_CS, LOW);
    writeByte(d);
    digitalWrite(LCD_CS, HIGH);
  }

  void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    if (rotation & 1) { y0 += 35; y1 += 35; }
    else { x0 += 35; x1 += 35; }
    writeCommand(0x2A);
    writeData(x0>>8); writeData(x0&0xFF);
    writeData(x1>>8); writeData(x1&0xFF);
    writeCommand(0x2B);
    writeData(y0>>8); writeData(y0&0xFF);
    writeData(y1>>8); writeData(y1&0xFF);
  }

  uint8_t rotation = 0;
};

ST7789_Parallel *gfx = new ST7789_Parallel(TFT_WIDTH, TFT_HEIGHT);
// Two ordinary white LEDs indicate the active control family. GPIO43 lights
// whenever a PARK, DRIVE, or LIFT preset is selected; GPIO44 lights in MANUAL.
bool displayedManualLight = false;
// The preset lamp "breathes" while target pressures are being reached, then
// gives three quick confirmation flashes before it remains steadily lit.
bool presetLedConfirming = false;
uint32_t presetLedConfirmStartedMs = 0;
// GPIO44 is used exclusively by the external PRESET indicator.
bool presetLedSelfTest = true;
uint32_t presetLedSelfTestStartedMs = 0;

void sendCommand(const uint8_t *p, size_t n);
void stopAll();
void tune(uint8_t w);
void choosePreset(uint8_t p);
void valve(uint8_t w,bool fill);
void line(uint8_t n,const char *s,bool selected);
void top(const char *s);
void foot(const char *s);
void value(char *out,size_t n,float f);
void home();
void manual();
void menu();
void picker();
void editor();
void calibration();
void settings();
void bright();
void setupEspNow();
void serviceEspNow();
void sendDiscovery();
bool addControllerPeer(const uint8_t *mac);
Key key();
void saveTarget();
void action(Key k);
void render();
void handleMenuButton();
void handleModeButton();
void handleHomeButtons();
void handleForceExhaust();
void updateForceExhaustFrames();
void handleAutoPairValves();
void handleManualValves();
void compressorMenu();
void drawCompressorValue(uint8_t row, bool selected);
void saveCompressorThreshold(uint8_t which);
void sendSettingsUpdate();
void applyControllerSettings();
void transitionTo(uint8_t newPage, uint8_t newCursor = 0);

Preferences prefs;
uint8_t controllerMac[6] = {0,0,0,0,0,0};
bool controllerPeerKnown = false;
uint32_t lastControllerPacketMs = 0;
uint32_t lastDiscoveryMs = 0;
PressurePacket receivedPressure = {};
volatile bool telemetryPending = false;
SettingsFrame receivedSettings = {};
volatile bool settingsFramePending = false;
SettingsFrame controllerSettings = {};
bool controllerSettingsLoaded = false;
const char *wheel[4] = {"FL", "FR", "RL", "RR"};
const char *preset[3] = {"PARK", "DRIVE", "LIFT"};
float pressure[5] = {NAN,NAN,NAN,NAN,NAN};
uint16_t target[3][4]={{20,20,20,20},{55,55,55,55},{80,80,80,80}};
bool linked=false, scanning=false, sleepMode=false;
bool manualMode=false;

// The historical BLE state labels are retained for the on-screen connection
// wording; ESP-NOW updates this value in serviceEspNow().
BleState bleState = BLE_SEARCHING;
volatile bool telemetry=false;
uint8_t page=0, cursor=0, active=0, editPreset=0, editWheel=0, calSensor=0, brightness=100;
// These two settings are independent from the screen backlight. They are
// saved in flash so the chosen LED level and beep level survive a restart.
uint8_t ledBrightness=100, buzzerVolume=70;
uint16_t calValue=0;
// Compressor thresholds are displayed in bar ×10. The controller is the
// authority and stores the values permanently after each B1 42 command.
uint16_t compressorOnBar10=110, compressorOffBar10=145;
uint8_t compressorRow=0;
bool menuTransitionPending=false;
int8_t heldWheel=-1;
int8_t heldValveButton=-1;
// Direction of the currently held MANUAL valve: true fills (blue frame),
// false exhausts (red frame).
bool heldValveFill = true;
// AUTO preset screen: one of the four pair buttons may temporarily override
// the controller while it is physically held. -1 means no override is active.
int8_t autoPairHeld = -1;       // 0 = FL/FR, 1 = RL/RR
bool autoPairFill = false;
bool autoPairBlinkVisible = false;
uint32_t autoPairBlinkMs = 0;
// GPIO3 is a deliberate all-wheel emergency exhaust only on the AUTO home
// screen. While it is held, every wheel card border blinks together in red.
bool forceExhaustActive = false;
bool forceExhaustBlinkVisible = false;
uint32_t forceExhaustBlinkMs = 0;
uint32_t lastInput=0;
// Time at which the controller link was last known to be available. A remote
// that cannot reconnect for 30 seconds turns its display off and deep-sleeps.
uint32_t controllerLostSinceMs = 0;
// USB raises the board's battery-sense input above the normal LiPo range.
// Cache that state so the controller-loss timeout runs only on battery power.
bool usbPowerPresent = false;
uint32_t lastPowerSourceCheckMs = 0;
// A level-low EXT1 wake source must be HIGH continuously before sleep.
// Keeping this timestamp prevents a bouncing or held switch from causing
// an immediate sleep/wake loop.
uint32_t allWakeButtonsReleasedSinceMs = 0;
uint32_t wakeInputLockUntilMs = 0;
bool modeButtonWasDown=false;
bool menuButtonWasDown=false;
bool menuButtonLongHandled=false;
uint32_t menuButtonDownMs=0;
uint32_t lastTelemetryPaintMs=0;
uint32_t lastBatteryPaintMs=0;
int displayedBatteryPercent=-1;
int displayedBatteryFillPercent=-1;
bool displayedBatteryCharging=false;
uint32_t batteryChargeAnimationMs=0;
uint8_t batteryChargeAnimationPercent=0;
bool batteryChargeKnownFull=false;
int displayedBleState=-1;
int displayedSignalBars=-1;
int displayedManualMode=-1;
int displayedSleepSeconds=-1;
int displayedSleepBarInnerW=-1;
int displayedTenth[5] = {-9999, -9999, -9999, -9999, -9999};
int displayedTankSegments = -1;

bool presetRunActive = false;
bool presetStopSent = false;
bool wheelTargetDetected[4] = {false, false, false, false};
bool wheelBlinkStarted[4] = {false, false, false, false};
bool wheelTargetComplete[4] = {false, false, false, false};
bool wheelLowPressureAlarm[4] = {false, false, false, false};
uint32_t wheelTargetDetectedMs[4] = {0, 0, 0, 0};
uint32_t wheelAlarmBlinkMs[4] = {0, 0, 0, 0};
bool wheelBlinkVisible[4] = {false, false, false, false};
uint32_t presetFrameBlinkMs = 0;
bool presetFrameBlinkVisible = false;
float wheelLockedPressure[4] = {NAN, NAN, NAN, NAN};
uint32_t manualFrameBlinkMs = 0;
bool manualFrameBlinkVisible = false;
uint32_t buzzerOffMs = 0;
uint32_t nextLowPressureBeepMs = 0;
uint32_t nextSleepWarningBeepMs = 0;

void sendCommand(const uint8_t *p, size_t n) {
  // Commands are permitted only after fresh telemetry authenticated the peer.
  if (!controllerPeerKnown || !linked || n == 0 || n > 250) return;
  const esp_err_t result = esp_now_send(controllerMac, p, n);
  if (result != ESP_OK) {
    linked = false;
    bleState = BLE_RETRYING;
  }
}

// A passive piezo needs an alternating signal, not a steady HIGH level.
// The caller chooses the pitch so normal actions and pressure warnings are
// clearly different without needing another physical sounder.
uint8_t presetLedDuty() {
  return (uint16_t)ledBrightness * 255U / 100U;
}

uint8_t buzzerDuty() {
  // A passive piezo is loudest near a 50% square wave. This limits its drive
  // duty from 5% to 50%, which gives a practical menu-controlled volume range.
  return 13U + ((uint16_t)buzzerVolume * 115U / 100U);
}

void startBeep(uint16_t frequencyHz, uint16_t durationMs) {
  ledcWriteTone(0, frequencyHz);
  ledcWrite(0, buzzerDuty());
  buzzerOffMs = millis() + durationMs;
}

void updateBuzzer() {
  const uint32_t now = millis();
  if (buzzerOffMs != 0 && (int32_t)(now - buzzerOffMs) >= 0) {
    ledcWriteTone(0, 0);
    buzzerOffMs = 0;
  }
}

void updateButtonLights(bool force) {
  const bool manualLightActive = manualMode;
  if (!force && manualLightActive == displayedManualLight) return;
  displayedManualLight = manualLightActive;
  // The MANUAL lamp is animated in updatePresetLightAnimation().  This helper
  // only makes an immediate safe hand-over between the two physical LEDs.
  if (manualLightActive) {
    // Start the MANUAL lamp immediately on a mode change. The regular loop
    // then continues the smooth breathing animation from this visible state.
    const uint8_t maximum = presetLedDuty();
    ledcWrite(2, 0);
    ledcWrite(1, maximum / 10U);
  } else {
    ledcWrite(1, 0);
    // A completed preset must leave its indicator off, not steadily lit.
    if (!presetRunActive && !presetLedConfirming) ledcWrite(2, 0);
  }
}

void updatePresetLightAnimation() {
  const uint32_t now = millis();
  // Hardware check: after each restart the GPIO44 LED makes three clear
  // flashes. This happens before BLE and preset logic, so it proves the wire,
  // resistor, LED polarity, and selected GPIO independently of the controller.
  if (presetLedSelfTest) {
    const uint32_t elapsed = now - presetLedSelfTestStartedMs;
    if (elapsed < 1200U) {
      ledcWrite(2, ((elapsed / 200U) % 2U) == 0U ? presetLedDuty() : 0);
    } else {
      presetLedSelfTest = false;
      updateButtonLights(true);
    }
    return;
  }
  if (manualMode) {
    // In MANUAL both external white LEDs blink together slowly.  They use the
    // same brightness setting and remain synchronised for the whole mode.
    const bool on = (now % 1600U) < 800U;
    const uint8_t duty = on ? presetLedDuty() : 0;
    ledcWrite(2, duty);  // GPIO44
    ledcWrite(1, duty);  // GPIO43
    return;
  }
  if (presetRunActive) {
    // A 1.2-second triangle wave looks like a gentle fade, not a sharp blink.
    const uint32_t phase = now % 1200U;
    const uint32_t ramp = phase < 600U ? phase : 1200U - phase;
    const uint8_t maximum = presetLedDuty();
    ledcWrite(2, maximum / 10U + ((uint16_t)maximum * 9U * ramp) / 6000U);
    return;
  }
  if (presetLedConfirming) {
    const uint32_t elapsed = now - presetLedConfirmStartedMs;
    if (elapsed < 720U) {
      // Three fast on/off flashes: 120 ms bright, then 120 ms dark.
      ledcWrite(2, ((elapsed / 120U) % 2U) == 0U ? presetLedDuty() : 0);
    } else {
      presetLedConfirming = false;
      // The three confirmation flashes are the end indication. Once the
      // valves are closed and the preset is settled, turn this LED off.
      ledcWrite(2, 0);
    }
  }
}

void stopAll() { const uint8_t p[]={0xB1,0x2F}; sendCommand(p,sizeof(p)); }
void tune(uint8_t w) { if(w>3)return; const uint8_t p[]={0xB1,0x45,active,w}; sendCommand(p,sizeof(p)); }
void choosePreset(uint8_t p) {
  manualMode = false;
  active = p;
  prefs.putUChar("preset", p);
  presetRunActive = true;
  presetStopSent = false;
  presetLedConfirming = false;
  presetFrameBlinkMs = millis();
  presetFrameBlinkVisible = true;
  if (page == 0) updatePresetLabel();
  updateButtonLights();
  for (uint8_t i = 0; i < 4; ++i) {
    wheelTargetDetected[i] = false;
    wheelBlinkStarted[i] = false;
    wheelTargetComplete[i] = false;
    wheelLowPressureAlarm[i] = false;
    wheelTargetDetectedMs[i] = 0;
    wheelAlarmBlinkMs[i] = 0;
    wheelBlinkVisible[i] = false;
    wheelLockedPressure[i] = NAN;
    if (page == 0) {
      const int x = (i & 1) ? 112 : 6;
      const int y = (i > 1) ? 77 : 6;
      drawWheelArrivalState(i, x, y);
    }
    tune(i);
  }
}
void valve(uint8_t w,bool fill) { if(w>3)return; const uint8_t p[]={0xB1,(uint8_t)(0x20+w*2+(fill?0:1))}; sendCommand(p,sizeof(p)); }

void line(uint8_t n,const char *s,bool selected) {
  int y=33+n*24;
  if(selected){gfx->fillRoundRect(4,y-2,312,21,6,ORANGE);gfx->setTextColor(BG);}
  else gfx->setTextColor(FG);
  gfx->setTextSize(1);
  gfx->setCursor(11,y+5);
  gfx->print(s);
}

void menuLine(uint8_t n, const char *s, bool selected) {
  const int y = 35 + n * 24;
  gfx->setTextSize(1);
  int16_t x1, y1; uint16_t textW, textH;
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &textW, &textH);
  const int blockW = textW + 24;
  const uint16_t blockColor = selected ? ORANGE : 0x21C9;
  gfx->fillRoundRect(8, y - 3, blockW, 19, 7, blockColor);
  gfx->drawRoundRect(8, y - 3, blockW, 19, 7, selected ? ORANGE : DIM);
  gfx->setTextColor(selected ? BG : FG);
  gfx->setCursor(20, y + 3);
  gfx->print(s);
}

bool usbPowerConnected() {
  uint32_t totalMv = 0;
  for (uint8_t i = 0; i < 4; ++i) totalMv += analogReadMilliVolts(BATTERY_ADC);
  // The T-Display-S3 battery divider reads outside the 3.0–4.4 V LiPo
  // range when USB-C powers the board. Treat that as external USB power.
  return (totalMv / 4U) * 2U > 4400U;
}

int batteryPercent() {
  // BAT_ADC (GPIO4) sits after a 2:1 divider. Average enough samples to
  // reject display/Wi-Fi noise, then round only after restoring battery volts.
  uint32_t totalMv = 0;
  for (uint8_t i = 0; i < 16; ++i) {
    totalMv += analogReadMilliVolts(BATTERY_ADC);
    delay(1);
  }
  const int batteryMv = (int)((totalMv * 2U + 8U) / 16U);

  // With USB-C attached this board routes the USB/charger supply to BAT_ADC,
  // so it cannot reveal the LiPo state. The caller shows charging instead.
  if (batteryMv < 3000 || batteryMv > 4400) return -1;

  if (batteryMv >= 4200) return 100;
  if (batteryMv >= 4100) return 90 + (batteryMv - 4100) * 10 / 100;
  if (batteryMv >= 4000) return 75 + (batteryMv - 4000) * 15 / 100;
  if (batteryMv >= 3850) return 45 + (batteryMv - 3850) * 30 / 150;
  if (batteryMv >= 3700) return 20 + (batteryMv - 3700) * 25 / 150;
  if (batteryMv >= 3500) return 5 + (batteryMv - 3500) * 15 / 200;
  return constrain((batteryMv - 3200) * 5 / 300, 0, 5);
}

void updateBatteryChargeAnimation() {
  const int measuredPercent = batteryPercent();
  if (!usbPowerPresent) {
    batteryChargeKnownFull = false;
    batteryChargeAnimationPercent = 0;
    return;
  }
  // When the battery divider remains readable at 4.15 V or above, the pack
  // is full and the icon becomes a steady full battery. Some boards expose
  // USB voltage on this ADC while charging; in that case the icon keeps its
  // honest moving charging animation because no charge-complete pin is wired.
  if (measuredPercent >= 100) {
    batteryChargeKnownFull = true;
    batteryChargeAnimationPercent = 100;
    return;
  }
  batteryChargeKnownFull = false;
  const uint32_t now = millis();
  if (now - batteryChargeAnimationMs >= 260U) {
    batteryChargeAnimationMs = now;
    batteryChargeAnimationPercent += 10;
    if (batteryChargeAnimationPercent > 100) batteryChargeAnimationPercent = 0;
  }
}

void drawBottomStatus(bool force = false) {
  if (page != 0) return;
  const int pct = batteryPercent();
  const bool charging = usbPowerPresent;
  const int shownBatteryFill = charging ? (batteryChargeKnownFull ? 100 : batteryChargeAnimationPercent) : pct;
  const int state = (int)bleState;
  // ESP-NOW does not expose per-packet RSSI through Arduino's receive API.
  // A fresh controller reply therefore shows four bars; no reply shows none.
  const int bars = linked ? 4 : 0;
  const uint16_t signalColor = LIME;
  if (force) gfx->drawFastHLine(5, 150, 310, DIM);
  if (force || state != displayedBleState) {
    displayedBleState = state;
    gfx->fillRect(7, 153, 96, 15, BG);
    const char *label = bleState == BLE_CONNECTED ? "CONNECTED" : bleState == BLE_CONNECTING ? "CONNECTING" : bleState == BLE_RETRYING ? "RETRYING" : "SEARCHING";
    gfx->setTextSize(1); gfx->setTextColor(bleState == BLE_CONNECTED ? GREEN : (bleState == BLE_CONNECTING ? ORANGE : RED));
    gfx->setCursor(8, 157); gfx->print(label);
  }
  if (force || bars != displayedSignalBars) {
    displayedSignalBars = bars;
    gfx->fillRect(270, 153, 44, 15, BG);
    for (uint8_t i=0; i<4; ++i) {
      gfx->fillRect(282 + i*7, 164 - i*3, 4, 3 + i*3, i < bars ? signalColor : DIM);
    }
  }
  // A compact orange pill shows the remaining controller-loss time. It is
  // redrawn only once per second, leaving the connection and mode labels still.
  const int sleepSeconds = linked || usbPowerPresent || controllerLostSinceMs == 0 ? -1 :
    max(0, 30 - (int)((millis() - controllerLostSinceMs) / 1000U));
  if (force || sleepSeconds != displayedSleepSeconds) {
    constexpr int sleepBarX = 105, sleepBarY = 155, sleepBarW = 76, sleepBarH = 10;
    const int innerW = sleepSeconds >= 0 ? (sleepBarW - 4) * sleepSeconds / 30 : 0;
    const bool redrawWholeBar = force || displayedSleepSeconds < 0 || sleepSeconds < 0;
    displayedSleepSeconds = sleepSeconds;
    if (redrawWholeBar) {
      gfx->fillRect(sleepBarX - 1, sleepBarY - 1, sleepBarW + 2, sleepBarH + 2, BG);
      if (sleepSeconds >= 0) {
        gfx->fillRoundRect(sleepBarX, sleepBarY, sleepBarW, sleepBarH, 5, 0x21C9);
        if (innerW > 0) gfx->fillRoundRect(sleepBarX + 2, sleepBarY + 2, innerW, sleepBarH - 4, 3, ORANGE);
        gfx->drawRoundRect(sleepBarX, sleepBarY, sleepBarW, sleepBarH, 5, ORANGE);
      }
    } else if (innerW < displayedSleepBarInnerW) {
      // Keep the oval outline and the already-drawn portion still. Only paint
      // the short piece that has expired back to the dark track color.
      gfx->fillRect(sleepBarX + 2 + innerW, sleepBarY + 2,
                    displayedSleepBarInnerW - innerW, sleepBarH - 4, 0x21C9);
      if (innerW > 0) {
        // Restore the round leading end without touching the rest of the bar.
        gfx->fillRoundRect(sleepBarX + 2 + innerW - 3, sleepBarY + 2, 4, sleepBarH - 4, 2, ORANGE);
      }
    }
    displayedSleepBarInnerW = innerW;
  }
  const int mode = manualMode ? 1 : 0;
  if (force || mode != displayedManualMode) {
    displayedManualMode = mode;
    gfx->fillRect(184, 153, 53, 15, BG);
    const char *modeLabel = manualMode ? "MANUAL" : "AUTO";
    gfx->setTextSize(1); gfx->setTextColor(LIME);
    int16_t x1, y1; uint16_t textW, textH;
    gfx->getTextBounds(modeLabel, 0, 0, &x1, &y1, &textW, &textH);
    gfx->setCursor(234 - textW, 157); gfx->print(modeLabel);
  }
  if (force || pct != displayedBatteryPercent || shownBatteryFill != displayedBatteryFillPercent ||
      charging != displayedBatteryCharging) {
    displayedBatteryPercent = pct;
    displayedBatteryFillPercent = shownBatteryFill;
    displayedBatteryCharging = charging;
    gfx->fillRect(237, 153, 34, 15, BG);
    gfx->drawRect(239, 155, 25, 11, FG);
    gfx->fillRect(264, 158, 2, 5, FG);
    if (shownBatteryFill >= 0) {
      const int fillWidth = shownBatteryFill == 0 ? 0 : max(1, shownBatteryFill * 21 / 100);
      if (fillWidth) {
        // Charging uses a vivid phone-style green; the empty part stays dark so the moving fill is clear.
        const uint16_t fillColor = charging ? 0x07E0 : (shownBatteryFill > 10 ? LIME : RED);
        gfx->fillRect(241, 157, fillWidth, 7, fillColor);
      }
    }
  }
}

// Status changes are incremental. Repainting the full bottom strip on every
// telemetry update was also repainting the sleep bar and made it visibly blink.
void drawBleStatus() { drawBottomStatus(false); }

void top(const char *s) {
  gfx->fillScreen(BG);
  gfx->setTextWrap(false);
  if (page != 0) {
    gfx->setTextSize(2);
    gfx->setTextColor(FG);
    gfx->setCursor(7,6);
    gfx->print(s);
    gfx->drawFastHLine(6,28,308,DIM);
  }
}

void foot(const char *s) {
  gfx->setTextSize(1);
  gfx->setTextColor(DIM);
  gfx->setCursor(7,158);
  gfx->print(s);
}

void value(char *out,size_t n,float f) {
  if(isnan(f))snprintf(out,n,"--.-");
  else snprintf(out,n,"%.1f",f);
}

void drawWheelArrivalState(uint8_t index, int x, int y) {
  // In MANUAL, only the held wheel gets a flashing outline: blue while
  // filling and red while exhausting. The card contents stay untouched.
  const bool manualHeld = manualMode && page == 0 && heldWheel == (int8_t)index;
  const bool autoPairCard = !manualMode && page == 0 && autoPairHeld >= 0 &&
    ((autoPairHeld == 0 && index < 2) || (autoPairHeld == 1 && index >= 2));
  const bool redBlink = (wheelLowPressureAlarm[index] || wheelBlinkStarted[index]) &&
                        wheelBlinkVisible[index];
  // Emergency exhaust takes visual priority: GPIO3 affects all four wheels,
  // so every card uses the same synchronous red flashing border.
  const uint16_t frameColor = forceExhaustActive ? (forceExhaustBlinkVisible ? RED : DIM) :
    (autoPairCard ? (autoPairBlinkVisible ? (autoPairFill ? BLUE : RED) : DIM) :
    (manualHeld ? (manualFrameBlinkVisible ? (heldValveFill ? BLUE : RED) : DIM) :
     (redBlink ? RED : (wheelTargetComplete[index] ? LIME : DIM))));
  for (uint8_t inset = 0; inset < 3; ++inset) {
    gfx->drawRoundRect(x + inset, y + inset, 103 - inset * 2, 66 - inset * 2,
                       9 - inset * 2, frameColor);
  }
}

void redrawWheelFrame(uint8_t index) {
  if (page != 0 || index > 3) return;
  const int x = (index & 1) ? 112 : 6;
  const int y = (index > 1) ? 77 : 6;
  drawWheelArrivalState(index, x, y);
}

void updateManualWheelFrame() {
  if (!manualMode || page != 0 || heldWheel < 0) return;
  const uint32_t now = millis();
  if (now - manualFrameBlinkMs < 250U) return;
  manualFrameBlinkMs = now;
  manualFrameBlinkVisible = !manualFrameBlinkVisible;
  redrawWheelFrame((uint8_t)heldWheel);
}

void wheelCard(uint8_t index, int x, int y) {
  gfx->fillRoundRect(x, y, 103, 66, 9, 0x21C9);
  drawWheelArrivalState(index, x, y);
  gfx->setTextSize(1); gfx->setTextColor(FG);
  int16_t labelX1, labelY1; uint16_t labelW, labelH;
  gfx->getTextBounds(wheel[index], 0, 0, &labelX1, &labelY1, &labelW, &labelH);
  // Left-side cards (FL/RL) label at top-right; right-side cards (FR/RR) at top-left.
  const int labelX = (index & 1) == 0 ? x + 94 - labelW - labelX1 : x + 9 - labelX1;
  gfx->setCursor(labelX, y + 10); gfx->print(wheel[index]);
  char reading[10]; value(reading, sizeof(reading), pressure[index]);
  // FL/RL use a left gauge; FR/RR use a right gauge. The track is 14 px wide.
  const bool leftGauge = (index & 1) == 0;
  const int gaugeX = leftGauge ? x + 8 : x + 81;
  const int numberX = leftGauge ? x + 29 : x + 11;
  gfx->setFont(&FreeSansBold9pt7b);
  gfx->setTextSize(2); gfx->setTextColor(ORANGE);
  gfx->setCursor(numberX, y + 51); gfx->print(reading);
  gfx->setFont();
  uint8_t level = isnan(pressure[index]) ? 0 : constrain((int)(pressure[index] * 10.0f), 0, 100);
  gfx->fillRoundRect(gaugeX, y + 17, 14, 40, 4, 0x1166);
  const int fillH = level * 36 / 100;
  if(fillH) gfx->fillRoundRect(gaugeX + 2, y + 55 - fillH, 10, fillH, 3, ORANGE);
}

void updateWheelReading(uint8_t index, int x, int y) {
  // Clear only the number/gauge region; the label and card outline remain still.
  gfx->fillRect(x + 7, y + 17, 89, 40, 0x21C9);
  char reading[10]; value(reading, sizeof(reading), pressure[index]);
  const bool leftGauge = (index & 1) == 0;
  const int gaugeX = leftGauge ? x + 8 : x + 81;
  const int numberX = leftGauge ? x + 29 : x + 11;
  gfx->setFont(&FreeSansBold9pt7b);
  gfx->setTextSize(2); gfx->setTextColor(ORANGE);
  gfx->setCursor(numberX, y + 51); gfx->print(reading);
  gfx->setFont();
  uint8_t level = isnan(pressure[index]) ? 0 : constrain((int)(pressure[index] * 10.0f), 0, 100);
  gfx->fillRoundRect(gaugeX, y + 17, 14, 40, 4, 0x1166);
  const int fillH = level * 36 / 100;
  if(fillH) gfx->fillRoundRect(gaugeX + 2, y + 55 - fillH, 10, fillH, 3, ORANGE);
}

void drawTankSegment(uint8_t segment, uint16_t color) {
  const int cx = 267, cy = 96, radius = 45;
  const float start = 142.0f, sweep = 256.0f;
  const float a = (start + segment * (sweep / 28.0f)) * DEG_TO_RAD;
  const float b = (start + segment * (sweep / 28.0f) + 3.6f) * DEG_TO_RAD;
  for (int r = radius - 2; r <= radius + 2; ++r) {
    gfx->drawLine(cx + (int)(cosf(a) * r), cy + (int)(sinf(a) * r),
                  cx + (int)(cosf(b) * r), cy + (int)(sinf(b) * r), color);
  }
}

int tankSegmentCount() {
  const int level = isnan(pressure[4]) ? 0 : constrain((int)lroundf(pressure[4] * 10.0f), 0, 150);
  return (level * 28 + 149) / 150;
}

void drawTankGauge() {
  const int cx = 267, cy = 96;
  gfx->fillRect(224, 51, 87, 96, BG);
  // Keep BAR 5 px below the nearest part of the outer gauge, then reserve
  // the centre exclusively for the large pressure reading.
  gfx->setTextSize(1); gfx->setTextColor(DIM);
  gfx->setCursor(cx - 9, cy - 26); gfx->print("BAR");
  const float shownTankBar = isnan(pressure[4]) ? NAN : constrain(pressure[4], 0.0f, 15.0f);
  char reading[10]; value(reading, sizeof(reading), shownTankBar);
  gfx->setFont(&FreeSansBold9pt7b);
  gfx->setTextSize(2); gfx->setTextColor(ORANGE);
  int16_t x1, y1; uint16_t w, h;
  gfx->getTextBounds(reading, 0, 0, &x1, &y1, &w, &h);
  // Centre the complete glyph bounds in the circular gauge; 15.0 still
  // clears both BAR above it and the TANK capsule below it.
  gfx->setCursor(cx - w / 2 - x1, cy - h / 2 - y1); gfx->print(reading);
  gfx->setFont();
  gfx->drawRoundRect(cx - 27, cy + 32, 54, 15, 7, DIM);
  gfx->setTextSize(1); gfx->setTextColor(FG);
  gfx->setCursor(cx - 12, cy + 36); gfx->print("TANK");
  displayedTankSegments = tankSegmentCount();
  for (uint8_t i = 0; i < displayedTankSegments; ++i) drawTankSegment(i, TANK_ORANGE);
}

void updateTankGauge() {
  const int newSegments = tankSegmentCount();
  if (displayedTankSegments < 0) displayedTankSegments = newSegments;
  if (newSegments > displayedTankSegments) {
    for (int i = displayedTankSegments; i < newSegments; ++i) drawTankSegment(i, TANK_ORANGE);
  } else if (newSegments < displayedTankSegments) {
    for (int i = newSegments; i < displayedTankSegments; ++i) drawTankSegment(i, BG);
  }
  displayedTankSegments = newSegments;
  const int cx = 267, cy = 96;
  gfx->fillRect(cx - 34, cy - 20, 68, 39, BG);
  // Keep BAR 5 px below the nearest part of the outer gauge, then reserve
  // the centre exclusively for the large pressure reading.
  gfx->setTextSize(1); gfx->setTextColor(DIM);
  gfx->setCursor(cx - 9, cy - 26); gfx->print("BAR");
  const float shownTankBar = isnan(pressure[4]) ? NAN : constrain(pressure[4], 0.0f, 15.0f);
  char reading[10]; value(reading, sizeof(reading), shownTankBar);
  gfx->setFont(&FreeSansBold9pt7b);
  gfx->setTextSize(2); gfx->setTextColor(ORANGE);
  int16_t x1, y1; uint16_t w, h;
  gfx->getTextBounds(reading, 0, 0, &x1, &y1, &w, &h);
  // Centre the complete glyph bounds in the circular gauge; 15.0 still
  // clears both BAR above it and the TANK capsule below it.
  gfx->setCursor(cx - w / 2 - x1, cy - h / 2 - y1); gfx->print(reading);
  gfx->setFont();
}

void drawPresetFrame() {
  const uint16_t frameColor = presetRunActive ?
    (presetFrameBlinkVisible ? RED : DIM) : (presetStopSent ? LIME : DIM);
  gfx->drawRoundRect(225, 7, 89, 38, 12, frameColor);
}

void updatePresetLabel() {
  gfx->fillRoundRect(225, 7, 89, 38, 12, 0x21C9);
  drawPresetFrame();
  const char *label = manualMode ? "MAN" : preset[active];
  gfx->setFont(&FreeSansBold9pt7b);
  gfx->setTextSize(1); gfx->setTextColor(LIME);
  int16_t x1, y1; uint16_t w, h;
  gfx->getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
  gfx->setCursor(225 + (89 - w) / 2 - x1, 7 + (38 - h) / 2 - y1);
  gfx->print(label);
  gfx->setFont();
}

void home() {
  for (uint8_t i = 0; i < 5; ++i) displayedTenth[i] = isnan(pressure[i]) ? -9999 : (int)lroundf(pressure[i] * 10.0f);
  displayedBatteryPercent = -1;
  displayedBatteryFillPercent = -1;
  displayedBatteryCharging = false;
  displayedBleState = -1;
  displayedSignalBars = -1;
  displayedManualMode = -1;
  displayedSleepSeconds = -1;
  displayedSleepBarInnerW = -1;
  top("");
  wheelCard(0, 6, 6);   wheelCard(1, 112, 6);
  wheelCard(2, 6, 77);  wheelCard(3, 112, 77);
  updatePresetLabel();
  drawTankGauge();
  drawBottomStatus(true);
}

void manual() {
  top("MANUAL");
  char s[30],v[10];
  for(uint8_t i=0;i<4;i++){
    value(v,sizeof(v),pressure[i]);
    snprintf(s,sizeof(s),"%s: %s bar",wheel[i],v);
    line(i,s,cursor==i);
  }
  foot("HOLD IN/OUT | OK next wheel");
}

void menu() {
  top("MENU");
  const char *m[]={"START PRESET","EDIT PRESETS","CALIBRATE","SETTINGS","COMPRESSOR"};
  for(uint8_t i=0;i<5;i++) menuLine(i, m[i], cursor==i);
  foot("UP/DN select  OK open");
}

void settings() {
  top("SETTINGS");
  const char *m[]={"SCREEN BRIGHTNESS","LED BRIGHTNESS","BUZZER VOLUME","SLEEP MODE"};
  for(uint8_t i=0;i<4;i++) menuLine(i, m[i], cursor==i);
  // Show the stored state on its own line so changing it needs no separate page.
  gfx->setTextSize(1); gfx->setTextColor(sleepMode ? LIME : DIM);
  gfx->setCursor(190, 35 + 3 * 24 + 3); gfx->print(sleepMode ? "ON" : "OFF");
  foot("UP/DN select  OK open  BACK menu");
}

void picker() {
  top("START PRESET");
  // Compact text-width cards leave the rest of the screen static.
  for(uint8_t i=0;i<3;i++) menuLine(i, preset[i], cursor==i);
  foot("UP/DN select  OK sends targets  BACK menu");
}

void drawEditorHeader() {
  gfx->fillRect(7, 34, 306, 19, BG);
  gfx->setTextSize(1); gfx->setTextColor(DIM);
  gfx->setCursor(11, 39); gfx->print("PRESET");
  gfx->setFont(&FreeSansBold9pt7b); gfx->setTextColor(LIME);
  gfx->setCursor(72, 48); gfx->print(preset[editPreset]);
  gfx->setFont();
}

void drawEditorWheel(uint8_t index, bool selected) {
  const int y = 57 + index * 23;
  gfx->fillRoundRect(7, y, 306, 20, 7, selected ? ORANGE : BG);
  if (!selected) gfx->drawRoundRect(7, y, 306, 20, 7, DIM);
  gfx->setTextSize(1); gfx->setTextColor(selected ? BG : FG);
  gfx->setCursor(17, y + 6); gfx->print(wheel[index]);
  char s[16]; snprintf(s, sizeof(s), "%.1f", target[editPreset][index] / 10.0f);
  int16_t x1, y1; uint16_t w, h;
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setCursor(285 - w, y + 6); gfx->print(s);
}

void editor() {
  top("EDIT PRESET");
  drawEditorHeader();
  for (uint8_t i = 0; i < 4; ++i) drawEditorWheel(i, i == editWheel);
  foot(controllerSettingsLoaded ? "UP/DN value  OK next wheel  BACK menu" : "WAIT FOR CONTROLLER SETTINGS");
}

void drawCalibrationSensor(uint8_t index, bool selected) {
  const int y = 39 + index * 22;
  gfx->fillRoundRect(7, y, 306, 19, 7, selected ? ORANGE : BG);
  if (!selected) gfx->drawRoundRect(7, y, 306, 19, 7, DIM);
  gfx->setTextSize(1); gfx->setTextColor(selected ? BG : FG);
  gfx->setCursor(17, y + 5); gfx->print(index < 4 ? wheel[index] : "TANK");
  char s[16]; snprintf(s, sizeof(s), "%.1f", calSensor == index ? calValue / 10.0f : (isnan(pressure[index]) ? 0.0f : pressure[index]));
  int16_t x1, y1; uint16_t w, h;
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setCursor(285 - w, y + 5); gfx->print(s);
}

void calibration() {
  top("CALIBRATE");
  for (uint8_t i = 0; i < 5; ++i) drawCalibrationSensor(i, i == calSensor);
  foot("UP/DN gauge  OK save/next  BACK menu");
}

void drawCompressorValue(uint8_t row, bool selected) {
  const int y = 48 + row * 38;
  gfx->fillRoundRect(7, y, 306, 31, 8, selected ? ORANGE : BG);
  if (!selected) gfx->drawRoundRect(7, y, 306, 31, 8, DIM);
  gfx->setTextSize(1); gfx->setTextColor(selected ? BG : FG);
  gfx->setCursor(17, y + 7); gfx->print(row == 0 ? "TURN ON" : "TURN OFF");
  const uint16_t value10 = row == 0 ? compressorOnBar10 : compressorOffBar10;
  char s[16]; snprintf(s, sizeof(s), "%.1f BAR", value10 / 10.0f);
  int16_t x1, y1; uint16_t w, h;
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setCursor(295 - w, y + 19); gfx->print(s);
}

void compressorMenu() {
  top("COMPRESSOR");
  drawCompressorValue(0, compressorRow == 0);
  drawCompressorValue(1, compressorRow == 1);
  foot("UP/DN value  OK row  BACK menu");
}

void saveCompressorThreshold(uint8_t) { sendSettingsUpdate(); }

void sendSettingsUpdate() {
  if (!controllerSettingsLoaded || !linked) return;
  uint8_t packet[50] = {0xB1, 0x43};
  size_t at = 2;
  for (uint8_t i = 0; i < 4; ++i, at += 2) { packet[at] = controllerSettings.wheelLimitDbar[i] & 0xFF; packet[at + 1] = controllerSettings.wheelLimitDbar[i] >> 8; }
  const uint16_t fixed[] = {controllerSettings.tankLimitDbar, controllerSettings.compressorOnDbar,
    controllerSettings.targetToleranceDbar, controllerSettings.autoTimeoutSec,
    controllerSettings.compressorMaxRunSec, controllerSettings.manualTimeoutMs};
  for (uint8_t i = 0; i < 6; ++i, at += 2) { packet[at] = fixed[i] & 0xFF; packet[at + 1] = fixed[i] >> 8; }
  for (uint8_t p = 0; p < 3; ++p) for (uint8_t w = 0; w < 4; ++w, at += 2) {
    packet[at] = target[p][w] & 0xFF; packet[at + 1] = target[p][w] >> 8;
  }
  const uint32_t checksum = checkBytes(packet, 46);
  memcpy(&packet[46], &checksum, sizeof(checksum));
  sendCommand(packet, sizeof(packet));
}

void applyControllerSettings() {
  if (!settingsFramePending) return;
  SettingsFrame snapshot;
  noInterrupts();
  memcpy(&snapshot, &receivedSettings, sizeof(snapshot));
  settingsFramePending = false;
  interrupts();
  if (snapshot.marker != 0xA2 || snapshot.version != 1 ||
      snapshot.check != checkBytes((const uint8_t *)&snapshot, offsetof(SettingsFrame, check))) return;
  controllerSettings = snapshot;
  memcpy(target, snapshot.presetDbar, sizeof(target));
  compressorOnBar10 = snapshot.compressorOnDbar;
  compressorOffBar10 = snapshot.tankLimitDbar;
  controllerSettingsLoaded = true;
  if (page == 4) editor();
  else if (page == 10) compressorMenu();
}

void transitionTo(uint8_t newPage, uint8_t newCursor) {
  page = newPage;
  cursor = newCursor;
  menuTransitionPending = true;
  render();
}

void bright() {
  top("SCREEN BRIGHTNESS");
  char s[16];
  snprintf(s,sizeof(s),"%u%%",brightness);
  line(1,s,true);
  foot("UP/DOWN change  BACK menu");
}

void ledBright() {
  top("LED BRIGHTNESS");
  char s[16];
  snprintf(s,sizeof(s),"%u%%",ledBrightness);
  line(1,s,true);
  foot("UP/DOWN change  BACK menu");
}

void volume() {
  top("BUZZER VOLUME");
  char s[16];
  snprintf(s,sizeof(s),"%u%%",buzzerVolume);
  line(1,s,true);
  foot("UP/DOWN change  BACK menu");
}

void updatePresetArrival() {
  const uint32_t now = millis();

  if (presetRunActive && now - presetFrameBlinkMs >= 250U) {
    presetFrameBlinkMs = now;
    presetFrameBlinkVisible = !presetFrameBlinkVisible;
    if (page == 0) drawPresetFrame();
  }

  for (uint8_t i = 0; i < 4; ++i) {
    if (wheelTargetComplete[i] && !wheelLowPressureAlarm[i] &&
        !isnan(wheelLockedPressure[i]) && !isnan(pressure[i]) &&
        pressure[i] < wheelLockedPressure[i] - 0.5f) {
      wheelLowPressureAlarm[i] = true;
      wheelAlarmBlinkMs[i] = now;
      wheelBlinkVisible[i] = true;
      // Start the repeating warning immediately, then repeat every 0.5 s
      // for as long as at least one completed wheel remains 0.5 bar low.
      nextLowPressureBeepMs = now;
      if (page == 0) {
        const int x = (i & 1) ? 112 : 6;
        const int y = (i > 1) ? 77 : 6;
        drawWheelArrivalState(i, x, y);
      }
    }
  }

  bool allComplete = true;
  for (uint8_t i = 0; i < 4; ++i) {
    const int x = (i & 1) ? 112 : 6;
    const int y = (i > 1) ? 77 : 6;
    const float targetBar = target[active][i] / 10.0f;
    const bool inTargetWindow = !isnan(pressure[i]) && fabsf(pressure[i] - targetBar) <= 0.2f;

    if (wheelLowPressureAlarm[i]) {
      if (inTargetWindow) {
        wheelLowPressureAlarm[i] = false;
        wheelBlinkVisible[i] = false;
        if (page == 0) drawWheelArrivalState(i, x, y);
      } else if (now - wheelAlarmBlinkMs[i] >= 250U) {
        wheelAlarmBlinkMs[i] = now;
        wheelBlinkVisible[i] = !wheelBlinkVisible[i];
        if (page == 0) drawWheelArrivalState(i, x, y);
      }
    }

    if (!wheelTargetComplete[i] && !wheelBlinkStarted[i]) {
      if (!inTargetWindow) {
        wheelTargetDetected[i] = false;
        wheelTargetDetectedMs[i] = 0;
      } else if (!wheelTargetDetected[i]) {
        wheelTargetDetected[i] = true;
        wheelTargetDetectedMs[i] = now;
      } else if (now - wheelTargetDetectedMs[i] >= 2000U) {
        wheelLowPressureAlarm[i] = false;
        wheelLockedPressure[i] = pressure[i];
        wheelBlinkStarted[i] = true;
        wheelTargetDetectedMs[i] = now;
        wheelBlinkVisible[i] = true;
        if (page == 0) drawWheelArrivalState(i, x, y);
      }
    }

    if (wheelBlinkStarted[i] && !wheelTargetComplete[i]) {
      const bool visible = ((now - wheelTargetDetectedMs[i]) / 250U) % 2U == 0U;
      if (visible != wheelBlinkVisible[i]) {
        wheelBlinkVisible[i] = visible;
        if (page == 0) drawWheelArrivalState(i, x, y);
      }
      if (now - wheelTargetDetectedMs[i] >= 2000U) {
        wheelTargetComplete[i] = true;
        wheelBlinkVisible[i] = false;
        if (page == 0) drawWheelArrivalState(i, x, y);
      }
    }
    if (!wheelTargetComplete[i]) allComplete = false;
  }

  if (allComplete && !presetStopSent) {
    stopAll();
    startBeep(4000, 160);  // One clearly audible confirmation beep when the preset settles.
    presetStopSent = true;
    presetRunActive = false;
    presetLedConfirming = true;
    presetLedConfirmStartedMs = now;
    presetFrameBlinkVisible = false;
    if (page == 0) drawPresetFrame();
  }
}

void updateHomeTelemetry() {
  updatePresetArrival();
  for (uint8_t i = 0; i < 5; ++i) {
    const int tenth = isnan(pressure[i]) ? -9999 : (int)lroundf(pressure[i] * 10.0f);
    if (tenth == displayedTenth[i]) continue;
    displayedTenth[i] = tenth;
    if (i < 4) {
      const int x = (i & 1) ? 112 : 6;
      const int y = (i > 1) ? 77 : 6;
      updateWheelReading(i, x, y);
    } else {
      updateTankGauge();
      drawBleStatus();
    }
  }
}

void render(){
  // A brief three-band wipe makes page changes clear without continuously
  // animating static screens or telemetry cards.
  if (menuTransitionPending) {
    for (uint8_t band = 0; band < 3; ++band) {
      gfx->fillRect(0, band * 57, 320, 57, band == 1 ? 0x21C9 : BG);
      delay(18);
    }
    menuTransitionPending = false;
  }
  if(page==0)home();
  else if(page==1)manual();
  else if(page==2)menu();
  else if(page==3)picker();
  else if(page==4)editor();
  else if(page==5)calibration();
  else if(page==6)settings();
  else if(page==7)bright();
  else if(page==8)ledBright();
  else if(page==9)volume();
  else if(page==10)compressorMenu();
  else { page=0; cursor=0; home(); }
}

bool addControllerPeer(const uint8_t *mac) {
  if (controllerPeerKnown && memcmp(controllerMac, mac, 6) == 0) return true;
  if (controllerPeerKnown) esp_now_del_peer(controllerMac);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = true;
  memcpy(peer.lmk, ESPNOW_LMK, sizeof(ESPNOW_LMK));
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  memcpy(controllerMac, mac, 6);
  controllerPeerKnown = true;
  return true;
}

void onEspNowReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
  if (mac_addr == nullptr || data == nullptr) return;

  // The controller deliberately answers discovery in the clear, then changes
  // this MAC address to the shared encrypted ESP-NOW key.  Complete that
  // two-step handshake before accepting telemetry; otherwise the controller
  // never receives E1 02 and correctly keeps every valve closed.
  if (len == 2 && data[0] == 0xE1 && data[1] == 0x81) {
    bleState = BLE_CONNECTING;
    if (addControllerPeer(mac_addr)) {
      const EspNowDiscoveryPacket confirm = {0xE1, 0x02};
      esp_now_send(controllerMac, reinterpret_cast<const uint8_t *>(&confirm), sizeof(confirm));
    }
    return;
  }

  if (!controllerPeerKnown || memcmp(controllerMac, mac_addr, 6) != 0) return;
  if (len == (int)sizeof(SettingsFrame) && data[0] == 0xA2) {
    memcpy(&receivedSettings, data, sizeof(receivedSettings));
    settingsFramePending = true;
    lastControllerPacketMs = millis();
    return;
  }
  if (len != (int)sizeof(PressurePacket) || data[0] != 0xA1) return;
  // The callback only copies complete frames. Drawing remains in loop().
  memcpy(&receivedPressure, data, sizeof(receivedPressure));
  lastControllerPacketMs = millis();
  telemetryPending = true;
}

void onEspNowSend(const const uint8_t *, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS && linked) {
    linked = false;
    bleState = BLE_RETRYING;
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  if (esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE) != ESP_OK || esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed.");
    return;
  }
  esp_now_set_pmk(ESPNOW_LMK);
  esp_now_register_recv_cb(onEspNowReceive);
  esp_now_register_send_cb(onEspNowSend);
  esp_now_peer_info_t broadcastPeer = {};
  memcpy(broadcastPeer.peer_addr, ESPNOW_BROADCAST, 6);
  broadcastPeer.channel = ESPNOW_CHANNEL;
  broadcastPeer.encrypt = false;
  esp_now_add_peer(&broadcastPeer);
  bleState = BLE_SEARCHING;
}

void sendDiscovery() {
  const EspNowDiscoveryPacket hello = {0xE1, 0x01};
  esp_now_send(ESPNOW_BROADCAST, reinterpret_cast<const uint8_t *>(&hello), sizeof(hello));
}

void serviceEspNow() {
  const uint32_t now = millis();
  const bool fresh = controllerPeerKnown && (now - lastControllerPacketMs < CONTROLLER_TIMEOUT_MS);
  if (fresh != linked) {
    linked = fresh;
    bleState = fresh ? BLE_CONNECTED : BLE_RETRYING;
    drawBleStatus();
    if (!fresh) {
      // A rebooted controller starts with no encrypted peer. Drop our old
      // peer record as well, so its clear E1 81 discovery reply is never
      // tied to stale encryption state. The next reply rebuilds the peer and
      // sends encrypted E1 02 exactly like the first connection.
      if (controllerPeerKnown) esp_now_del_peer(controllerMac);
      controllerPeerKnown = false;
      controllerSettingsLoaded = false;
      memset(controllerMac, 0, sizeof(controllerMac));
      lastControllerPacketMs = 0;
      // Do not wait for the normal 500 ms discovery period after a controller
      // disappears. Start a new clear broadcast handshake immediately; this
      // lets a rebooted or briefly out-of-range controller answer E1 01 as
      // soon as it returns, then receive the encrypted E1 02 confirmation.
      lastDiscoveryMs = now;
      sendDiscovery();
    }
  }
  if (!fresh && now - lastDiscoveryMs >= DISCOVERY_INTERVAL_MS) {
    lastDiscoveryMs = now;
    bleState = controllerPeerKnown ? BLE_RETRYING : BLE_SEARCHING;
    sendDiscovery();
  }
  if (fresh && now - lastDiscoveryMs >= DISCOVERY_INTERVAL_MS) {
    lastDiscoveryMs = now;
    const EspNowDiscoveryPacket heartbeat = {0xE1, 0x02};
    esp_now_send(controllerMac, reinterpret_cast<const uint8_t *>(&heartbeat), sizeof(heartbeat));
  }
  applyControllerSettings();
  if (telemetryPending) {
    noInterrupts();
    PressurePacket packet = receivedPressure;
    telemetryPending = false;
    interrupts();
    for (uint8_t i = 0; i < 5; ++i) pressure[i] = packet.bar[i];
    telemetry = true;
  }
}

bool anyWakeButtonPressed() {
  // Each normally-open button connects its GPIO to GND only while pressed.
  // A LOW level at the instant EXT1 is armed would wake the ESP32 immediately.
  const uint8_t wakePins[] = {
    BTN_MENU, BTN_MODE, BTN_DRIVE, BTN_PARK, BTN_LIFT,
    BTN_FORCE_EXHAUST, BTN_FL_IN, BTN_FR_IN, BTN_FL_OUT, BTN_FR_OUT
  };
  for (uint8_t i = 0; i < sizeof(wakePins) / sizeof(wakePins[0]); ++i) {
    pinMode(wakePins[i], INPUT_PULLUP);
    if (digitalRead(wakePins[i]) == LOW) return true;
  }
  return false;
}

void enterControllerLostSleep() {
  // All external controls are normally-open buttons wired between their GPIO
  // and GND. EXT1 wakes the ESP32-S3 when any one of those lines goes LOW.
  constexpr uint64_t wakeButtonMask =
      (1ULL << BTN_MENU) | (1ULL << BTN_MODE) |
      (1ULL << BTN_DRIVE) | (1ULL << BTN_PARK) |
      (1ULL << BTN_LIFT) | (1ULL << BTN_FORCE_EXHAUST) |
      (1ULL << BTN_FL_IN) | (1ULL << BTN_FR_IN) |
      (1ULL << BTN_FL_OUT) | (1ULL << BTN_FR_OUT);

  // EXT1 is level-sensitive, not edge-sensitive. Require every line to remain
  // released for 750 ms; an 80 ms check leaves a race where contact bounce can
  // make the freshly armed wake source LOW immediately after deep sleep begins.
  const uint32_t now = millis();
  if (anyWakeButtonPressed()) {
    // Do not restart the one-minute controller-loss timer here. A LOW input
    // cannot safely arm level-sensitive wake, but restarting the timer made
    // one noisy or held button keep the remote awake forever.
    allWakeButtonsReleasedSinceMs = 0;
    return;
  }
  if (allWakeButtonsReleasedSinceMs == 0) {
    allWakeButtonsReleasedSinceMs = now;
    return;
  }
  if (now - allWakeButtonsReleasedSinceMs < 750U) return;

  // INPUT_PULLUP belongs to the normal GPIO block, which powers down in deep
  // sleep. Keep the RTC-domain pull-ups enabled instead: without them, long
  // button wires can float LOW the moment sleep starts and immediately wake
  // the remote again.
  const uint8_t rtcWakePins[] = {
    BTN_MENU, BTN_MODE, BTN_DRIVE, BTN_PARK, BTN_LIFT,
    BTN_FORCE_EXHAUST, BTN_FL_IN, BTN_FR_IN, BTN_FL_OUT, BTN_FR_OUT
  };
  for (uint8_t i = 0; i < sizeof(rtcWakePins) / sizeof(rtcWakePins[0]); ++i) {
    const gpio_num_t pin = (gpio_num_t)rtcWakePins[i];
    // RTC pull resistors persist through deep sleep only after this GPIO is
    // attached to the RTC domain and configured as an RTC input.
    rtc_gpio_init(pin);
    rtc_gpio_set_direction(pin, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pullup_en(pin);
    rtc_gpio_pulldown_dis(pin);
  }

  stopAll();
  // ESP-NOW has no scanner or connection object to stop before sleep.
  analogWrite(LCD_BL, 0);
  digitalWrite(LCD_POWER, LOW);
  Serial.flush();
  esp_sleep_enable_ext1_wakeup(wakeButtonMask, ESP_EXT1_WAKEUP_ANY_LOW);
  esp_deep_sleep_start();
}


Key key(){
  static uint8_t was[4] = {HIGH, HIGH, HIGH, HIGH};
  const uint8_t pin[4] = {BTN_UP, BTN_DOWN, BTN_OK, BTN_BACK};
  const Key mapped[4] = {UP, DOWN, KEYOK, BACK};
  for (uint8_t i = 0; i < 4; ++i) {
    const uint8_t now = digitalRead(pin[i]);
    if (was[i] == HIGH && now == LOW) { was[i] = now; return mapped[i]; }
    was[i] = now;
  }
  return NONE;
}

void saveTarget(){ sendSettingsUpdate(); }

void handleModeButton(){
  const bool down = digitalRead(BTN_MODE) == LOW;
  if (down && !modeButtonWasDown) {
    startBeep(4000, 35);
    modeButtonWasDown = true;
    stopAll();
    heldWheel = -1;
    heldValveButton = -1;
    manualMode = !manualMode;
    page = 0;
    cursor = 0;
    updatePresetLabel();
    updateButtonLights();
    drawBottomStatus();
  }
  if (!down) modeButtonWasDown = false;
}

void handleHomeButtons(){
  static uint8_t previous[3] = {HIGH, HIGH, HIGH};
  const uint8_t pins[3] = {BTN_PARK, BTN_DRIVE, BTN_LIFT};
  for (uint8_t i = 0; i < 3; ++i) {
    const uint8_t now = digitalRead(pins[i]);
    if (previous[i] == HIGH && now == LOW && !manualMode && page == 0) {
      startBeep(4000, 35);
      if (i == 0) choosePreset(0);       // GPIO11: PARK
      else if (i == 1) choosePreset(1);  // GPIO1: DRIVE
      else choosePreset(2);               // GPIO2: LIFT
    }
    previous[i] = now;
  }
}

void handleForceExhaust() {
  static bool wasDown = false;
  // GPIO3 is the RL exhaust hold button in MANUAL. The all-wheel exhaust
  // override is available only in AUTO, so it cannot steal that manual input.
  if (manualMode) {
    wasDown = false;
    forceExhaustActive = false;
    return;
  }
  const bool down = digitalRead(BTN_FORCE_EXHAUST) == LOW;
  if (down && !wasDown && page == 0) {
    startBeep(4000, 35);
    // This is an intentional emergency-style override: cancel an automatic
    // target run, then command all four exhaust valves while the button stays held.
    stopAll();
    presetRunActive = false;
    presetStopSent = true;
    forceExhaustActive = true;
    forceExhaustBlinkVisible = true;
    forceExhaustBlinkMs = millis();
    for (uint8_t i = 0; i < 4; ++i) {
      valve(i, false);
      redrawWheelFrame(i);
    }
    wasDown = true;
  } else if (!down && wasDown) {
    // Closing immediately on release prevents a stuck exhaust command.
    stopAll();
    forceExhaustActive = false;
    for (uint8_t i = 0; i < 4; ++i) redrawWheelFrame(i);
    wasDown = false;
  }
}

void updateForceExhaustFrames() {
  if (!forceExhaustActive || page != 0 || manualMode) return;
  const uint32_t now = millis();
  if (now - forceExhaustBlinkMs < 250U) return;
  forceExhaustBlinkMs = now;
  forceExhaustBlinkVisible = !forceExhaustBlinkVisible;
  // Only redraw the three-pixel outlines: the pressure readings and card
  // contents remain still during the held emergency exhaust.
  for (uint8_t i = 0; i < 4; ++i) redrawWheelFrame(i);
}

void handleAutoPairValves() {
  if (manualMode || page != 0) return;
  struct PairKey { uint8_t pin; uint8_t pair; bool fill; };
  // GPIO17 fills FL+FR; GPIO13 fills RL+RR. GPIO21 exhausts FL+FR;
  // GPIO16 exhausts RL+RR. These commands are valid only in AUTO/preset view.
  static const PairKey keys[] = {
    {BTN_FL_IN,  0, true}, {BTN_FR_IN,  1, true},
    {BTN_FL_OUT, 0, false}, {BTN_FR_OUT, 1, false}
  };
  int8_t pressed = -1;
  for (uint8_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
    if (digitalRead(keys[i].pin) == LOW) { pressed = i; break; }
  }
  if (pressed < 0) {
    if (autoPairHeld >= 0) {
      const int8_t oldPair = autoPairHeld;
      stopAll();  // release closes both forced valves immediately
      autoPairHeld = -1;
      for (uint8_t wheelIndex = oldPair == 0 ? 0 : 2; wheelIndex < (oldPair == 0 ? 2 : 4); ++wheelIndex) redrawWheelFrame(wheelIndex);
    }
    return;
  }
  const PairKey &wanted = keys[pressed];
  if (autoPairHeld == wanted.pair && autoPairFill == wanted.fill) {
    if (millis() - autoPairBlinkMs >= 250U) {
      autoPairBlinkMs = millis();
      autoPairBlinkVisible = !autoPairBlinkVisible;
      const uint8_t first = autoPairHeld == 0 ? 0 : 2;
      redrawWheelFrame(first);
      redrawWheelFrame(first + 1);
    }
    return;
  }
  if (autoPairHeld >= 0) stopAll();
  autoPairHeld = wanted.pair;
  autoPairFill = wanted.fill;
  autoPairBlinkMs = millis();
  autoPairBlinkVisible = true;
  startBeep(4000, 35);
  const uint8_t first = wanted.pair == 0 ? 0 : 2;
  valve(first, wanted.fill);
  valve(first + 1, wanted.fill);
  redrawWheelFrame(first);
  redrawWheelFrame(first + 1);
}

void handleManualValves(){
  if (!manualMode || page != 0) return;
  struct ValveKey { uint8_t pin; uint8_t wheel; bool fill; };
  static const ValveKey keys[] = {
    {BTN_FL_IN, 0, true}, {BTN_FR_IN, 1, true},
    {BTN_RL_IN, 2, true}, {BTN_RR_IN, 3, true},
    {BTN_FL_OUT, 0, false}, {BTN_FR_OUT, 1, false},
    {BTN_RL_OUT, 2, false}, {BTN_RR_OUT, 3, false}
  };
  int8_t pressed = -1;
  for (uint8_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
    if (digitalRead(keys[i].pin) == LOW) { pressed = i; break; }
  }
  if (pressed < 0) {
    if (heldValveButton >= 0) {
      const int8_t oldWheel = heldWheel;
      stopAll(); heldWheel = -1; heldValveButton = -1; heldValveFill = true;
      redrawWheelFrame((uint8_t)oldWheel);
    }
    return;
  }
  const ValveKey &wanted = keys[pressed];
  if (heldValveButton != wanted.pin) {
    startBeep(4000, 35);
    const int8_t oldWheel = heldWheel;
    if (heldValveButton >= 0) stopAll();
    heldWheel = wanted.wheel;
    heldValveButton = wanted.pin;
    heldValveFill = wanted.fill;
    manualFrameBlinkMs = millis();
    manualFrameBlinkVisible = true;
    if (oldWheel >= 0 && oldWheel != heldWheel) redrawWheelFrame((uint8_t)oldWheel);
    redrawWheelFrame(wanted.wheel);
    valve(wanted.wheel, wanted.fill);
  }
}

void handleMenuButton(){
  const bool down = digitalRead(BTN_MENU) == LOW;
  const uint32_t now = millis();
  if (down && !menuButtonWasDown) {
    startBeep(4000, 35);
    menuButtonWasDown = true;
    menuButtonLongHandled = false;
    menuButtonDownMs = now;
    return;
  }
  if (!down && menuButtonWasDown) {
    const bool shortPress = !menuButtonLongHandled;
    menuButtonWasDown = false;
    if (shortPress && page != 2) {
      page = 2;
      cursor = 0;
      // Start the menu inactivity timer when the menu actually opens.
      // Without this, a remote left on the home screen for a minute could
      // return from MENU immediately on its first frame.
      lastInput = now;
      render();
    }
  }
}

void action(Key k){
  if(k==NONE)return;
  startBeep(4000, 35);
  if(k==PRESET_KEY){
    choosePreset((active + 1) % 3);
    if (page == 0) updatePresetLabel();
    else { page=0; cursor=0; render(); }
    return;
  }
  if(k==PARK_KEY){ choosePreset(0); if(page==0) updatePresetLabel(); else {page=0; cursor=0; render();} return; }
  if(k==DRIVE_KEY){ choosePreset(1); if(page==0) updatePresetLabel(); else {page=0; cursor=1; render();} return; }
  if(k==LIFT_KEY){ choosePreset(2); if(page==0) updatePresetLabel(); else {page=0; cursor=2; render();} return; }
  if(page==1 && k==IN_KEY){
    heldWheel=cursor;
    heldValveButton=BTN_IN;
    valve(cursor,true);
    render();
    return;
  }
  if(page==1 && k==OUT_KEY){
    heldWheel=cursor;
    heldValveButton=BTN_OUT;
    valve(cursor,false);
    render();
    return;
  }
  if(page==0){
    if(k==UP&&cursor)cursor--;
    else if(k==DOWN&&cursor<4)cursor++;
    else if(k==KEYOK){
    }
    else if(k==NEXT){
      if(cursor<3)choosePreset(cursor);
      else if(cursor==3){manualMode=true;page=1;cursor=0;}
      else{page=2;cursor=0;}
    }
  } else if(page==1){
    if(k==KEYOK)cursor=(cursor+1)%4;
    else if(k==NEXT){tune(cursor<2?0:2);tune(cursor<2?1:3);}
    else if(k==BACK){stopAll();heldWheel=-1;heldValveButton=-1;page=0;cursor=0;}
  } else if(page==2){
    // GPIO1 and GPIO3 only move the highlight. Redraw the two small menu
    // rows instead of repainting the complete display, which prevents flicker.
    static const char *menuItems[] = {
      "START PRESET", "EDIT PRESETS", "CALIBRATE", "SETTINGS", "COMPRESSOR"
    };
    if ((k == UP && cursor > 0) || (k == DOWN && cursor < 4)) {
      const uint8_t previousCursor = cursor;
      cursor += (k == UP) ? -1 : 1;
      menuLine(previousCursor, menuItems[previousCursor], false);
      menuLine(cursor, menuItems[cursor], true);
      return;
    }
    // GPIO2 is OK: enter the highlighted function.
    if (k == KEYOK) {
      if(cursor==0){transitionTo(3, 0);}
      else if(cursor==1){editPreset=editWheel=0; transitionTo(4);}
      else if(cursor==2){
        calSensor=0;
        calValue=isnan(pressure[0]) ? 0 : (uint16_t)constrain((int)lroundf(pressure[0]*10.0f),0,150);
        transitionTo(5);
      }
      else if(cursor==3) {transitionTo(6, 0);}
      else { compressorRow=0; transitionTo(10); }
    } else if (k == BACK) {
      // GPIO11 leaves the top menu and returns to the home screen.
      page=0;
      cursor=0;
    }
  } else if(page==3){
    // Move just the two compact preset labels; do not repaint START PRESET.
    if ((k == UP && cursor > 0) || (k == DOWN && cursor < 2)) {
      const uint8_t oldCursor = cursor;
      cursor += (k == UP) ? -1 : 1;
      menuLine(oldCursor, preset[oldCursor], false);
      menuLine(cursor, preset[cursor], true);
      return;
    }
    if(k==KEYOK){choosePreset(cursor); transitionTo(0);}
    else if(k==BACK){transitionTo(2, 0);}
  } else if(page==4){
    if (!controllerSettingsLoaded) return;
    if (k == UP && target[editPreset][editWheel] < controllerSettings.wheelLimitDbar[editWheel]) {
      target[editPreset][editWheel] += 1;
      saveTarget();
      drawEditorWheel(editWheel, true);
      return;
    }
    if (k == DOWN && target[editPreset][editWheel] >= 1) {
      target[editPreset][editWheel] -= 1;
      saveTarget();
      drawEditorWheel(editWheel, true);
      return;
    }
    if (k == KEYOK) {
      const uint8_t oldWheel = editWheel;
      editWheel++;
      if (editWheel >= 4) {
        editWheel = 0;
        editPreset = (editPreset + 1) % 3;
        drawEditorHeader();
        for (uint8_t i = 0; i < 4; ++i) drawEditorWheel(i, i == editWheel);
      } else {
        drawEditorWheel(oldWheel, false);
        drawEditorWheel(editWheel, true);
      }
      return;
    }
    if (k == BACK) { transitionTo(2, 1); }
  } else if(page==5){
    if (k == UP && calValue < 150) {
      calValue++;
      drawCalibrationSensor(calSensor, true);
      return;
    }
    if (k == DOWN && calValue > 0) {
      calValue--;
      drawCalibrationSensor(calSensor, true);
      return;
    }
    if (k == KEYOK) {
      stopAll();
      const uint8_t p[]={0xB1,0x41,calSensor,(uint8_t)calValue,(uint8_t)(calValue>>8)};
      sendCommand(p,sizeof(p));
      const uint8_t oldSensor = calSensor;
      calSensor = (calSensor + 1) % 5;
      calValue = isnan(pressure[calSensor]) ? 0 : (uint16_t)constrain((int)lroundf(pressure[calSensor] * 10.0f), 0, 150);
      drawCalibrationSensor(oldSensor, false);
      drawCalibrationSensor(calSensor, true);
      return;
    }
    if(k==BACK){transitionTo(2, 2);}
  } else if(page==10) {
    if (k == UP) {
      if (!controllerSettingsLoaded) return;
      uint16_t &selected = compressorRow == 0 ? compressorOnBar10 : compressorOffBar10;
      const uint16_t maximum = compressorRow == 0 ? compressorOffBar10 - 1 : 150;
      if (selected < maximum) { selected++; if (compressorRow == 0) controllerSettings.compressorOnDbar = selected; else controllerSettings.tankLimitDbar = selected; saveCompressorThreshold(compressorRow); drawCompressorValue(compressorRow, true); }
      return;
    }
    if (k == DOWN) {
      if (!controllerSettingsLoaded) return;
      uint16_t &selected = compressorRow == 0 ? compressorOnBar10 : compressorOffBar10;
      const uint16_t minimum = compressorRow == 0 ? 0 : compressorOnBar10 + 1;
      if (selected > minimum) { selected--; if (compressorRow == 0) controllerSettings.compressorOnDbar = selected; else controllerSettings.tankLimitDbar = selected; saveCompressorThreshold(compressorRow); drawCompressorValue(compressorRow, true); }
      return;
    }
    if (k == KEYOK) {
      const uint8_t oldRow = compressorRow;
      compressorRow = (compressorRow + 1) % 2;
      drawCompressorValue(oldRow, false);
      drawCompressorValue(compressorRow, true);
      return;
    }
    if (k == BACK) { transitionTo(2, 4); return; }
  } else if(page==6) {
    static const char *settingItems[] = {"SCREEN BRIGHTNESS", "LED BRIGHTNESS", "BUZZER VOLUME", "SLEEP MODE"};
    if ((k == UP && cursor > 0) || (k == DOWN && cursor < 3)) {
      const uint8_t oldCursor = cursor;
      cursor += (k == UP) ? -1 : 1;
      menuLine(oldCursor, settingItems[oldCursor], false);
      menuLine(cursor, settingItems[cursor], true);
      // Restore the ON/OFF word because the selected-row repaint covers it.
      gfx->setTextSize(1); gfx->setTextColor(sleepMode ? LIME : DIM);
      gfx->setCursor(190, 35 + 3 * 24 + 3); gfx->print(sleepMode ? "ON" : "OFF");
      return;
    }
    if (k == KEYOK) {
      // Open the selected setting explicitly.  Keeping each destination in
      // this switch prevents a cursor value from ever falling through to an
      // unrelated page.
      if (cursor == 0) {
        page = 7;  // SCREEN BRIGHTNESS
        render();
      } else if (cursor == 1) {
        page = 8;  // LED BRIGHTNESS
        render();
      } else if (cursor == 2) {
        page = 9;  // BUZZER VOLUME
        render();
      } else {
        sleepMode = !sleepMode;
        prefs.putBool("sleep", sleepMode);
        // Only redraw the status word, leaving the menu still.
        gfx->fillRect(188, 35 + 3 * 24 - 2, 40, 19, sleepMode ? ORANGE : 0x21C9);
        gfx->setTextSize(1); gfx->setTextColor(sleepMode ? LIME : DIM);
        gfx->setCursor(190, 35 + 3 * 24 + 3); gfx->print(sleepMode ? "ON" : "OFF");
      }
      return;
    }
    if (k == BACK) { transitionTo(2, 3); }
  } else if(page==7) {
    if(k==UP&&brightness<100)brightness+=10;
    else if(k==DOWN&&brightness>10)brightness-=10;
    else if(k==BACK){transitionTo(6, 0);}
    prefs.putUChar("bright",brightness);
    analogWrite(LCD_BL,(uint16_t)brightness*255/100);
  } else if(page==8) {
    if(k==UP&&ledBrightness<100)ledBrightness+=10;
    else if(k==DOWN&&ledBrightness>10)ledBrightness-=10;
    else if(k==BACK){transitionTo(6, 1);}
    prefs.putUChar("led",ledBrightness);
    updateButtonLights(true);
  } else {
    if(k==UP&&buzzerVolume<100)buzzerVolume+=10;
    else if(k==DOWN&&buzzerVolume>10)buzzerVolume-=10;
    else if(k==BACK){transitionTo(6, 2);}
    prefs.putUChar("volume",buzzerVolume);
    // Apply the new level immediately if a warning beep is still sounding.
    if (buzzerOffMs != 0) ledcWrite(0, buzzerDuty());
  }
  render();
}

void setup(){
  Serial.begin(115200);
  setCpuFrequencyMhz(240);
  Serial.printf("CPU clock: %u MHz\n", getCpuFrequencyMhz());
  pinMode(LCD_POWER,OUTPUT);
  digitalWrite(LCD_POWER,HIGH);
  delay(20);
  pinMode(LCD_BL,OUTPUT);
  analogWrite(LCD_BL,255);
  // Stop boot diagnostics before the external LED tests begin.
  Serial.end();
  // Hardware proof before PWM, BLE, or the display code can affect it: GPIO44
  // holds the external preset LED fully on for two seconds at every restart.
  // This is a direct 3.3 V output test, not a preset-status animation.
  pinMode(PRESET_LED_PIN, OUTPUT);
  digitalWrite(PRESET_LED_PIN, HIGH);
  delay(2000);
  digitalWrite(PRESET_LED_PIN, LOW);
  // Attach GPIO18 to a PWM tone generator for the passive piezo buzzer.
  ledcSetup(0, 2400, 8);
  ledcAttachPin(BUZZER_PIN, 0);
  ledcWriteTone(0, 0);
  // PWM lets both ordinary white LEDs use the menu-selected brightness.
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MANUAL_LED_PIN, 1);
  ledcWrite(1, 0);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(PRESET_LED_PIN, 2);
  ledcWrite(2, 0);
  presetLedSelfTestStartedMs = millis();
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_ADC, ADC_11db);
  const uint8_t buttons[]={BTN_MENU,BTN_MODE,BTN_DRIVE,BTN_PARK,BTN_LIFT,BTN_FORCE_EXHAUST,BTN_FL_IN,BTN_FR_IN,BTN_FL_OUT,BTN_FR_OUT};
  for(uint8_t i=0;i<10;i++)pinMode(buttons[i],INPUT_PULLUP);
  gfx->begin();
  gfx->setRotation(3);
  gfx->fillScreen(BG);
  prefs.begin("airride",false);
  active=prefs.getUChar("preset",0);
  if(active>2)active=0;
  updateButtonLights(true);
  brightness=prefs.getUChar("bright",100);
  if(brightness<10||brightness>100)brightness=100;
  ledBrightness=prefs.getUChar("led",100);
  if(ledBrightness<10||ledBrightness>100)ledBrightness=100;
  buzzerVolume=prefs.getUChar("volume",70);
  if(buzzerVolume<10||buzzerVolume>100)buzzerVolume=70;
  // Vehicle settings are loaded from the controller after the encrypted
  // ESP-NOW link is established. These values only keep the menu drawable
  // while the controller is still connecting.
  compressorOnBar10=120;
  compressorOffBar10=150;
  analogWrite(LCD_BL,(uint16_t)brightness*255/100);
  // Preset pressures are owned and saved by the controller.
  setupEspNow();
  usbPowerPresent = usbPowerConnected();
  lastPowerSourceCheckMs = millis();
  // Begin the 30-second disconnect timer after every power-on or wake-up.
  // USB power clears it in loop(), so only battery operation can sleep.
  controllerLostSinceMs = millis();
  allWakeButtonsReleasedSinceMs = 0;
  // A button that woke the remote may still be held. Do not treat that same
  // hold as a menu or valve request after restart; wait for it to be released.
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
    wakeInputLockUntilMs = millis() + 150U;
    Serial.println("Woke from a button; waiting for release before accepting input.");
  }
  render();
}

void loop(){
  // Hold off button actions while the wake button is still physically down.
  if (wakeInputLockUntilMs != 0) {
    if (anyWakeButtonPressed()) wakeInputLockUntilMs = millis() + 150U;
    else if (millis() >= wakeInputLockUntilMs) wakeInputLockUntilMs = 0;
  }
  serviceEspNow();
  updateBuzzer();
  updatePresetLightAnimation();
  bool lowPressureAlarmActive = false;
  for (uint8_t i = 0; i < 4; ++i) lowPressureAlarmActive |= wheelLowPressureAlarm[i];
  if (lowPressureAlarmActive && (int32_t)(millis() - nextLowPressureBeepMs) >= 0) {
    // A lower pitch makes a real pressure-drop warning distinct from button taps.
    startBeep(1800, 120);
    nextLowPressureBeepMs = millis() + 500U;
  }
  if (!lowPressureAlarmActive) nextLowPressureBeepMs = 0;
  if (millis() - lastPowerSourceCheckMs >= 1000U) {
    lastPowerSourceCheckMs = millis();
    usbPowerPresent = usbPowerConnected();
  }
  updateBatteryChargeAnimation();
  // Keep the remote awake indefinitely while USB-C powers it. The 30-second
  // loss countdown, display-off action, and deep sleep apply only to battery.
  if (linked || usbPowerPresent) {
    controllerLostSinceMs = 0;
    nextSleepWarningBeepMs = 0;
  } else if (controllerLostSinceMs == 0) {
    controllerLostSinceMs = millis();
    nextSleepWarningBeepMs = controllerLostSinceMs + 5000U;
  } else if (millis() - controllerLostSinceMs >= 30000U) {
    enterControllerLostSleep();
  } else if (!lowPressureAlarmActive && nextSleepWarningBeepMs != 0 &&
             (int32_t)(millis() - nextSleepWarningBeepMs) >= 0) {
    // While running on the battery, remind the user every five seconds that
    // the disconnected remote is about to turn its screen off and sleep.
    startBeep(4000, 70);
    nextSleepWarningBeepMs += 5000U;
  }
  if (wakeInputLockUntilMs == 0) {
    handleMenuButton();
    handleModeButton();
    if (page == 0) handleForceExhaust();
    if (manualMode && page == 0) {
      // In MANUAL every valve button is a hold-to-open control. GPIO3 maps
      // to RL exhaust here; only AUTO uses it for forced all-wheel exhaust.
      handleManualValves();
    } else if (!manualMode && page == 0) {
      // The four pair buttons are hold-to-open overrides while a preset is
      // selected. GPIO3 retains its separate all-wheel emergency exhaust role.
      handleAutoPairValves();
      if (digitalRead(BTN_FORCE_EXHAUST) != LOW && autoPairHeld < 0) handleHomeButtons();
    } else {
      Key k=key();
      if(k!=NONE){lastInput=millis();action(k);}
    }
  }
  // The 30-second countdown changes once a second while the controller is absent.
  // Battery sampling stays at ten seconds so it does not add visible churn.
  if (page == 0 && ((usbPowerPresent && millis() - lastBatteryPaintMs >= 260U) ||
                    (!usbPowerPresent && !linked && millis() - lastBatteryPaintMs >= 1000U) ||
                    (!usbPowerPresent && linked && millis() - lastBatteryPaintMs >= 10000U))) {
    lastBatteryPaintMs = millis();
    drawBottomStatus();
  }
  if(telemetry && millis()-lastTelemetryPaintMs>=250){
    telemetry=false;
    lastTelemetryPaintMs=millis();
    if(page==0) updateHomeTelemetry();
    else if(page==1) render();
  }
  if(page==0) {
    updatePresetArrival();
    updateManualWheelFrame();
    updateForceExhaustFrames();
  }
  if(page!=0&&page!=1&&millis()-lastInput>60000){page=0;cursor=0;render();}
  delay(15);
}