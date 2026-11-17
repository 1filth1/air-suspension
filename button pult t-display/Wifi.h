#pragma once

// BombProtocol 0.6.0 includes this Arduino-ESP32 header with a lowercase
// second letter. The ESP32 core provides it as WiFi.h; keep this compatibility
// shim local to this project so the unused dependency cannot stop the remote
// firmware from compiling.
#include <WiFi.h>
