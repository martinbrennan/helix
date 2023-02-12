
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"

#include "helix.h"

#define LCD_HOST       SPI2_HOST
#define LCDHEIGHT 240
#define LCDWIDTH 240
#define PARALLEL_LINES 20

#define LCD_BK_LIGHT_OFF_LEVEL 0
#define LCD_BK_LIGHT_ON_LEVEL 1

static uint16_t *myRect;

void lcdInit() {
	
  gpio_config_t bk_gpio_config = {.mode = GPIO_MODE_OUTPUT,
                                  .pin_bit_mask = 1ULL<<9};		// backlight GPIO 9
  // Initialize the GPIO of backlight
  ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
  
    spi_bus_config_t buscfg = {
        .sclk_io_num = 6,						// SCLK
        .mosi_io_num = 7,						// MOSI
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PARALLEL_LINES * LCDWIDTH * 2 + 8
    };

    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));  
  

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = 4,						// DC
        .cs_gpio_num = 5,						// CS
        .pclk_hz = 20000000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };  

    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = 8,					// RESET
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
//        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    // Initialize the LCD configuration
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));


    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
    ESP_ERROR_CHECK(gpio_set_level(9, LCD_BK_LIGHT_OFF_LEVEL));



    // Reset the display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle,true));

    // Initialize LCD panel
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // Turn on the screen
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // Swap x and y axis (Different LCD screens may need different options)
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));

    // Turn on backlight (Different LCD screens may need different levels)
    ESP_ERROR_CHECK(gpio_set_level(9, LCD_BK_LIGHT_ON_LEVEL));
    
    myRect = heap_caps_malloc(40 * 40 * sizeof(uint16_t), MALLOC_CAP_DMA);    

	int n;
//	for (n=0;n<(40*40);n++) myRect[n] = 0xF800;		// red
	for (n=0;n<(40*40);n++) myRect[n] = 0x001F;		// 
//	for (n=0;n<(40*40);n++) myRect[n] = 0x00F8;
//	for (n=0;n<(40*40);n++) myRect[n] = 0x8000;		// white ish
//	for (n=0;n<(40*40);n++) myRect[n] = 0x4000;		// white ish
//	for (n=0;n<(40*40);n++) myRect[n] = 0x0000;		// white ish
//	for (n=0;n<(40*40);n++) myRect[n] = 0xFFFF;		// white ish
	
    esp_lcd_panel_draw_bitmap(panel_handle, 40, 40, 80, 80, myRect);	


  
}	
