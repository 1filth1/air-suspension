#pragma once
#include <LovyanGFX.hpp>
#include <lgfx/v1/platforms/esp32s3/Panel_RGB.hpp>
#include <lgfx/v1/platforms/esp32s3/Bus_RGB.hpp>

// Native RGB configuration from LovyanGFX's ESP32-8048S043 profile.
// The display is used at 800 x 480 with no software rotation.
class LGFX_Sunton8048S043 : public lgfx::LGFX_Device {
  lgfx::v1::Panel_RGB _panel_instance;
  lgfx::v1::Bus_RGB _bus_instance;

public:
  LGFX_Sunton8048S043() {
    auto bus_cfg = _bus_instance.config();
    bus_cfg.panel = &_panel_instance;
    bus_cfg.pin_d0 = GPIO_NUM_8;
    bus_cfg.pin_d1 = GPIO_NUM_3;
    bus_cfg.pin_d2 = GPIO_NUM_46;
    bus_cfg.pin_d3 = GPIO_NUM_9;
    bus_cfg.pin_d4 = GPIO_NUM_1;
    bus_cfg.pin_d5 = GPIO_NUM_5;
    bus_cfg.pin_d6 = GPIO_NUM_6;
    bus_cfg.pin_d7 = GPIO_NUM_7;
    bus_cfg.pin_d8 = GPIO_NUM_15;
    bus_cfg.pin_d9 = GPIO_NUM_16;
    bus_cfg.pin_d10 = GPIO_NUM_4;
    bus_cfg.pin_d11 = GPIO_NUM_45;
    bus_cfg.pin_d12 = GPIO_NUM_48;
    bus_cfg.pin_d13 = GPIO_NUM_47;
    bus_cfg.pin_d14 = GPIO_NUM_21;
    bus_cfg.pin_d15 = GPIO_NUM_14;
    bus_cfg.pin_henable = GPIO_NUM_40;
    bus_cfg.pin_vsync = GPIO_NUM_41;
    bus_cfg.pin_hsync = GPIO_NUM_39;
    bus_cfg.pin_pclk = GPIO_NUM_42;
    bus_cfg.freq_write = 14000000;
    bus_cfg.hsync_polarity = 0;
    bus_cfg.hsync_front_porch = 8;
    bus_cfg.hsync_pulse_width = 4;
    bus_cfg.hsync_back_porch = 16;
    bus_cfg.vsync_polarity = 0;
    bus_cfg.vsync_front_porch = 8;
    bus_cfg.vsync_pulse_width = 4;
    bus_cfg.vsync_back_porch = 8;
    bus_cfg.pclk_active_neg = true;
    _bus_instance.config(bus_cfg);

    _panel_instance.setBus(&_bus_instance);
    auto panel_cfg = _panel_instance.config();
    panel_cfg.memory_width = 800;
    panel_cfg.memory_height = 480;
    panel_cfg.panel_width = 800;
    panel_cfg.panel_height = 480;
    panel_cfg.offset_x = 0;
    panel_cfg.offset_y = 0;
    _panel_instance.config(panel_cfg);

    auto detail_cfg = _panel_instance.config_detail();
    detail_cfg.use_psram = 1;
    _panel_instance.config_detail(detail_cfg);
    setPanel(&_panel_instance);
  }
};
