idf_component_register(SRCS "main.c" "class_driver.c" "lcd.c" "audio.c" "es8388.c" "i2c_bus.c" "utils.c" "musicbrainz.c" "sha1.c" "base64.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_lcd usb lvgl)
