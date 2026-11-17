idf_component_register(
  SRCS "${CMAKE_CURRENT_LIST_DIR}/../main.c"
  INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/.."
  REQUIRES driver esp_adc esp_wifi nvs_flash
)
