/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include "driver/gpio.h"
#include <string.h>
#include "helix.h"



void hostSel() {
  // zero-initialize the config structure.
  gpio_config_t io_conf = {};
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set, GPIO20
  io_conf.pin_bit_mask = 1ULL << 18;
  // disable pull-down mode
  io_conf.pull_down_en = 0;
  // disable pull-up mode
  io_conf.pull_up_en = 0;
  // configure GPIO with the given settings
  gpio_config(&io_conf);
  gpio_set_level(18, 1);
  ESP_LOGE("MJB", "hostSel()");
}

void hostOn() {
  // zero-initialize the config structure.
  gpio_config_t io_conf = {};
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set, GPIO20
  io_conf.pin_bit_mask = 1ULL << 17;
  // disable pull-down mode
  io_conf.pull_down_en = 0;
  // disable pull-up mode
  io_conf.pull_up_en = 0;
  // configure GPIO with the given settings
  gpio_config(&io_conf);
  gpio_set_level(17, 1);
  ESP_LOGE("MJB", "hostOn()");
}

void vbusDevEn() {
  // zero-initialize the config structure.
  gpio_config_t io_conf = {};
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set, GPIO20
  io_conf.pin_bit_mask = 1ULL << 12;
  // disable pull-down mode
  io_conf.pull_down_en = 0;
  // disable pull-up mode
  io_conf.pull_up_en = 0;
  // configure GPIO with the given settings
  gpio_config(&io_conf);
  gpio_set_level(12, 1);
  ESP_LOGE("MJB", "vbusDevEn()");
}



#define DAEMON_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3

extern void class_driver_task(void *arg);

static const char *TAG = "DAEMON";

static void host_lib_daemon_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;

    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    //Signal to the class driver task that the host library is installed
    xSemaphoreGive(signaling_sem);
    vTaskDelay(10); //Short delay to let client task spin up

    bool has_clients = true;
    bool has_devices = true;
    while (has_clients || has_devices ) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            has_clients = false;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            has_devices = false;
        }
    }
    ESP_LOGI(TAG, "No more clients and devices");

    //Uninstall the USB Host Library
    ESP_ERROR_CHECK(usb_host_uninstall());
    //Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

static void myTask(void *arg)
{
	char lbuf[100];
	int lblen = 0;
	while (1){
		int c = fgetc (stdin);
		if (c >= 0) {
			if (c == 0xA) {
				printf ("\n");
				lbuf[lblen] = 0;
				lblen = 0;
				if (strcasecmp(lbuf,"ready") == 0) testUnitReady ();
				else if (strcasecmp(lbuf,"sense") == 0) requestSense ();
				else if (strcasecmp(lbuf,"read") == 0) readBlocks (100000+75 * 60,1000);
				else if (strcasecmp(lbuf,"toc") == 0) {
					int t = readToc (0,1);
					t = readToc (0,t+1);
					printf ("readToc result %d\n",t);
					printf ("Disk length result %d\n",getTrackStart(t));					
				}	
				else if (strcasecmp(lbuf,"check") == 0) {
					testUnitReady ();
					int r = testUnitReady ();					
					printf ("check result %d\n",r);
				}
			}
			else if (lblen < 99){
				printf ("%c",c);
				lbuf[lblen++] = c;
			}		
		}	
		vTaskDelay (10);
	}	
}


void app_main(void)
{
	
	hostSel();
	hostOn();
	vbusDevEn();	
	
    SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();

    TaskHandle_t daemon_task_hdl;
    TaskHandle_t class_driver_task_hdl;
    TaskHandle_t my_task_hdl;
    //Create daemon task
    xTaskCreatePinnedToCore(host_lib_daemon_task,
                            "daemon",
                            4096,
                            (void *)signaling_sem,
                            DAEMON_TASK_PRIORITY,
                            &daemon_task_hdl,
                            0);
    //Create the class driver task
    xTaskCreatePinnedToCore(class_driver_task,
                            "class",
                            4096,
                            (void *)signaling_sem,
                            CLASS_TASK_PRIORITY,
                            &class_driver_task_hdl,
                            0);

    xTaskCreatePinnedToCore(myTask,
                            "myTask",
                            4096,
                            NULL,
                            2,
                            &my_task_hdl,
                            0);

    vTaskDelay(10);     //Add a short delay to let the tasks run

	audioInit();
	startAudioThread();	

	lcdInit ();	

    //Wait for the tasks to complete
    for (int i = 0; i < 2; i++) {
        xSemaphoreTake(signaling_sem, portMAX_DELAY);
    }

    //Delete the tasks
    vTaskDelete(class_driver_task_hdl);
    vTaskDelete(daemon_task_hdl);
}