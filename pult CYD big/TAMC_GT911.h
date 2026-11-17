#pragma once

#include <Arduino.h>
#include <Wire.h>

// Compact GT911 I2C polling driver compatible with the ESP32-8048S043C panel.
// Constructor and fields intentionally match the common TAMC_GT911 interface.
#ifndef ROTATION_NORMAL
#define ROTATION_NORMAL 0
#endif

struct TP_Point {
  uint16_t x;
  uint16_t y;
  uint16_t size;
};

class TAMC_GT911 {
 public:
  TAMC_GT911(uint8_t sda, uint8_t scl, int8_t interruptPin, int8_t resetPin,
             uint16_t panelWidth, uint16_t panelHeight)
      : sda_(sda), scl_(scl), int_(interruptPin), rst_(resetPin),
        width_(panelWidth), height_(panelHeight) {}

  void begin() {
    // Reset and address-latch sequence for the GT911 fitted to ESP32-8048S043C.
    // INT must be LOW while RESET rises to choose address 0x5D.
    Wire.begin(sda_, scl_);
    Wire.setClock(400000);
    pinMode(rst_, OUTPUT);
    digitalWrite(rst_, LOW);
    if (int_ >= 0) {
      pinMode(int_, OUTPUT);
      digitalWrite(int_, LOW);
    }
    delay(20);
    // Keep RESET driven HIGH. Releasing this line as an input can leave it
    // floating on this panel and the GT911 then never produces touch records.
    digitalWrite(rst_, HIGH);
    delay(120);
    if (int_ >= 0) pinMode(int_, INPUT_PULLUP);
    delay(20);

    const uint8_t candidates[] = {0x5D, 0x14};
    uint8_t id[4] = {};
    for (uint8_t candidate : candidates) {
      address_ = candidate;
      if (read(0x8140, id, sizeof(id))) {
        Serial.printf("GT911: found at 0x%02X, id %.4s\n", address_, id);
        return;
      }
    }
    address_ = 0;
    Serial.println("GT911: not found at 0x5D or 0x14");
  }

  void setRotation(uint8_t rotation) { rotation_ = rotation; }

  void read() {
    touches = 0;
    isTouched = false;
    if (!address_) return;

    uint8_t status = 0;
    if (!read(0x814E, &status, 1)) {
      logFailure("I2C read failed");
      return;
    }
    if (!(status & 0x80)) return;

    const uint8_t count = status & 0x0F;
    if (count == 0) logEmptyReport();
    if (count > 0 && count <= 5) {
      uint8_t point[7];
      for (uint8_t i = 0; i < count; ++i) {
        // The first point starts at 0x814F: track ID, X, Y, then area.
        // Starting at 0x8150 drops the track ID and shifts both coordinates.
        if (!read((uint16_t)(0x814F + i * 8), point, sizeof(point))) break;
        points[touches].x = (uint16_t)point[1] | ((uint16_t)point[2] << 8);
        points[touches].y = (uint16_t)point[3] | ((uint16_t)point[4] << 8);
        points[touches].size = (uint16_t)point[5] | ((uint16_t)point[6] << 8);
        ++touches;
      }
      isTouched = touches > 0;
    }
    // Every ready report, including a release report, must be acknowledged.
    write(0x814E, 0x00);
  }

  uint8_t touches = 0;
  bool isTouched = false;
  TP_Point points[5] = {};

 private:
  bool read(uint16_t reg, uint8_t *data, size_t count) {
    if (!address_) return false;
    Wire.beginTransmission(address_);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(address_, (uint8_t)count) != count) {
      while (Wire.available()) Wire.read();
      return false;
    }
    for (size_t i = 0; i < count; ++i) data[i] = Wire.read();
    return true;
  }

  void write(uint16_t reg, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  void logFailure(const char *message) {
    const uint32_t now = millis();
    if (now - lastDiagnosticMs_ >= 1000) {
      lastDiagnosticMs_ = now;
      Serial.printf("GT911: %s at 0x%02X\\n", message, address_);
    }
  }

  void logEmptyReport() {
    const uint32_t now = millis();
    if (now - lastDiagnosticMs_ >= 1000) {
      lastDiagnosticMs_ = now;
      Serial.println("GT911: ready report has zero touch points");
    }
  }

  uint8_t sda_, scl_;
  int8_t int_, rst_;
  uint16_t width_, height_;
  uint8_t rotation_ = ROTATION_NORMAL;
  uint8_t address_ = 0;
  uint32_t lastDiagnosticMs_ = 0;
};
