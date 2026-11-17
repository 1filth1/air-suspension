#pragma once

// Use the ESP32 Arduino framework's ESP-NOW API. This local forwarding header
// prevents PlatformIO's dependency finder from selecting an unrelated library
// that also ships a file named esp_now.h.
#include_next <esp_now.h>
