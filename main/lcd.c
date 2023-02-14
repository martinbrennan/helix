
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include <stdio.h>

#include "esp_timer.h"

#include "helix.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"


#define LCD_HOST SPI2_HOST
#define LCDHEIGHT 240
#define LCDWIDTH 240
#define PARALLEL_LINES 20

#define LCD_BK_LIGHT_OFF_LEVEL 0
#define LCD_BK_LIGHT_ON_LEVEL 1

static uint16_t *myRect;

esp_lcd_panel_handle_t panel_handle = NULL;

void my_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area,
                 lv_color_t *color_p) {

//  printf("flush %d %d %d %d\n", area->x1, area->y1, area->x2, area->y2);

  esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2+1,
                            area->y2+1, color_p);

  lv_disp_flush_ready(disp_drv);
}

static void onTimer(void *arg) {
	
	lv_tick_inc(5);
	lv_timer_handler();
}	



void lcdInit() {

  gpio_config_t bk_gpio_config = {
      .mode = GPIO_MODE_OUTPUT, .pin_bit_mask = 1ULL << 9}; // backlight GPIO 9
  // Initialize the GPIO of backlight
  ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

  spi_bus_config_t buscfg = {.sclk_io_num = 6, // SCLK
                             .mosi_io_num = 7, // MOSI
                             .miso_io_num = -1,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1,
                             .max_transfer_sz =
                                 PARALLEL_LINES * LCDWIDTH * 2 + 8};

  // Initialize the SPI bus
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {
      .dc_gpio_num = 4, // DC
      .cs_gpio_num = 5, // CS
      .pclk_hz = 20000000,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .spi_mode = 0,
      .trans_queue_depth = 10,
  };

  // Attach the LCD to the SPI bus
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &io_config, &io_handle));

  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = 8, // RESET
      .rgb_endian = LCD_RGB_ENDIAN_BGR,
      //        .rgb_endian = LCD_RGB_ENDIAN_RGB,
      .bits_per_pixel = 16,
  };
  // Initialize the LCD configuration
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

  // Turn off backlight to avoid unpredictable display on the LCD screen while
  // initializing the LCD panel driver. (Different LCD screens may need
  // different levels)
  ESP_ERROR_CHECK(gpio_set_level(9, LCD_BK_LIGHT_OFF_LEVEL));

  // Reset the display
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));

  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));

  // Initialize LCD panel
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  
  // Mirror X
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle,false,false));   

  // Turn on the screen
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

  // Swap x and y axis (Different LCD screens may need different options)
  ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));

  // Turn on backlight (Different LCD screens may need different levels)
  ESP_ERROR_CHECK(gpio_set_level(9, LCD_BK_LIGHT_ON_LEVEL));

  myRect = heap_caps_malloc(40 * 40 * sizeof(uint16_t), MALLOC_CAP_DMA);
  
  
  int n;

  for (n = 0; n < (40 * 40); n++) myRect[n] = 0x0;
  for (n = 0; n < 240; n++){
  esp_lcd_panel_draw_bitmap(panel_handle, n, 0, n+1, 240, myRect);  
	}
  

  lv_init();

  /*A static or global variable to store the buffers*/
  static lv_disp_draw_buf_t disp_buf;

  /*Static or global buffer(s). The second buffer is optional*/
  static lv_color_t buf_1[LCDWIDTH * 10];
  static lv_color_t buf_2[LCDWIDTH * 10];

  /*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL
   * instead buf_2 */
  lv_disp_draw_buf_init(&disp_buf, buf_1, buf_2, LCDWIDTH * 10);

  static lv_disp_drv_t
      disp_drv; /*A variable to hold the drivers. Must be static or global.*/
  lv_disp_drv_init(&disp_drv);   /*Basic initialization*/
  disp_drv.draw_buf = &disp_buf; /*Set an initialized buffer*/
  disp_drv.flush_cb =
      my_flush_cb;        /*Set a flush callback to draw to the display*/
  disp_drv.hor_res = LCDWIDTH; /*Set the horizontal resolution in pixels*/
  disp_drv.ver_res = LCDHEIGHT; /*Set the vertical resolution in pixels*/

  lv_disp_t *disp;
  disp = lv_disp_drv_register(
      &disp_drv); /*Register the driver and save the created display objects*/
      
  const esp_timer_create_args_t periodic_timer_args = {
      //            .callback = &periodic_timer_callback,
      .callback = &onTimer,
      /* name is optional, but may help identify the timer when debugging */
      .name = "lvgl timer"};

  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  /* The timer has been created but is not running yet */

  /* Start the timers */
  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 5000)); 
  
//  lv_obj_t * screen = lv_scr_act();

	lv_color_t grey = lv_color_make (0x80, 0x80, 0x80);
	lv_color_t black = lv_color_make (0x0, 0x00, 0x0);
       
    lv_obj_set_style_bg_color (lv_scr_act(), black, LV_STATE_DEFAULT);   
       

    lv_obj_t * btn1 = lv_btn_create(lv_scr_act());
        lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);

    lv_obj_t * label;

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Button");
    lv_obj_center(label);
      
}
