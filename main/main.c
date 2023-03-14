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
				else if (strcasecmp(lbuf,"read") == 0) {
//					readBlocks (100000+75 * 60,1000);
					readBlocks (getTrackStart(0),1);
				}	
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
				else if (strcasecmp(lbuf,"eject") == 0) {
					int r = eject ();					
					printf ("eject result %d\n",r);
				}
				else if (strcasecmp(lbuf,"discid") == 0) {
					char buf[64];
					printf ("\n");
					computeDiscid (buf);
				}									
				else if (strcasecmp(lbuf,"load") == 0) {
					int r = load ();					
					printf ("load result %d\n",r);
				}
				else if (strcasecmp(lbuf,"play") == 0) {					
					printf ("startPlayingCD ()\n");
					startPlayingCD (getTrackStart(1));					
				}
				else if (strcasecmp(lbuf,"stop") == 0) {					
					printf ("stopPlayingCD\n");
					stopPlayingCD ();
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
	
	usbInit ();
	
    TaskHandle_t my_task_hdl;	

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
	startCDThread();

	lcdInit ();	

	waitForUSBToFinish ();
    
}
