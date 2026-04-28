idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES sx1262 ssd1306 esp_driver_uart
)
