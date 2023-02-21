idf_component_register(SRCS "main.c" "class_driver.c" "lcd.c" "audio.c" "es8388.c" "i2c_bus.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_lcd usb lvgl)
