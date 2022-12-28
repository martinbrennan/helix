/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include <string.h>

#define CLIENT_NUM_EVENT_MSG        5

#define ACTION_OPEN_DEV             0x01
#define ACTION_GET_DEV_INFO         0x02
#define ACTION_GET_DEV_DESC         0x04
#define ACTION_GET_CONFIG_DESC      0x08
#define ACTION_GET_STR_DESC         0x10
#define ACTION_CLOSE_DEV            0x20
#define ACTION_EXIT                 0x40

#define ACTION_MYTEST               0x80

typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
} class_driver_t;

static const char *TAG = "CLASS";

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            if (driver_obj->dev_addr == 0) {
                driver_obj->dev_addr = event_msg->new_dev.address;
                //Open the device next
                driver_obj->actions |= ACTION_OPEN_DEV;
            }
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            if (driver_obj->dev_hdl != NULL) {
                //Cancel any other actions and close the device next
                driver_obj->actions = ACTION_CLOSE_DEV;
            }
            break;
        default:
            //Should never occur
            abort();
    }
}

static void action_open_dev(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_addr != 0);
    ESP_LOGI(TAG, "Opening device at address %d", driver_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(driver_obj->client_hdl, driver_obj->dev_addr, &driver_obj->dev_hdl));
    //Get the device's information next
    driver_obj->actions &= ~ACTION_OPEN_DEV;
    driver_obj->actions |= ACTION_GET_DEV_INFO;
}

static void action_get_info(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device information");
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    ESP_LOGI(TAG, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
    ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
    //Todo: Print string descriptors

    //Get the device descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_INFO;
    driver_obj->actions |= ACTION_GET_DEV_DESC;
}

static void action_get_dev_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device descriptor");
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc));
    usb_print_device_descriptor(dev_desc);
    //Get the device's config descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_DESC;
    driver_obj->actions |= ACTION_GET_CONFIG_DESC;
}

static void action_get_config_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting config descriptor");
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
    usb_print_config_descriptor(config_desc, NULL);
    //Get the device's string descriptors next
    driver_obj->actions &= ~ACTION_GET_CONFIG_DESC;
    driver_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    if (dev_info.str_desc_manufacturer) {
        ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_manufacturer);
    }
    if (dev_info.str_desc_product) {
        ESP_LOGI(TAG, "Getting Product string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_product);
    }
    if (dev_info.str_desc_serial_num) {
        ESP_LOGI(TAG, "Getting Serial Number string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_serial_num);
    }
    //Nothing to do until the device disconnects
    
    printf ("MJB end of action_get_str_desc()\n"); 
    
    driver_obj->actions &= ~ACTION_GET_STR_DESC;
    driver_obj->actions |= ACTION_MYTEST;
}

unsigned char requestSenseCmd[31] = {
    0x55, 0x53, 0x42, 0x43, // byte 0-3: dCBWSignature
    0,    0,    0,    1,    // byte 4-7: dCBWTag
    0x12, 0,    0,    0,    // byte 8-11: dCBWDataTransferLength
    0x80,                   // byte 12: bmCBWFlags
    0,                      // byte 13: bit 7-4 Reserved(0), bCBWLUN
    0x06,                   // byte 14: bit 7-5 Reserved(0), bCBWCBLength
    0x03, 0,    0,    0,    // byte 15-30: CBWCommandBlock
    0x12, 0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0};


unsigned char inquiryCmd[31] = {
    0x55, 0x53, 0x42, 0x43, // byte 0-3:
                            // dCBWSignature
    0, 0, 0, 0,             // byte 4-7: dCBWTag
    0x24, 0, 0, 0,          // byte 8-11: dCBWDataTransferLength
    0x80,                   // byte 12: bmCBWFlags
    0,                      // byte 13: bit 7-4 Reserved(0), bCBWLUN
    0x06,                   // byte 14: bit 7-5 Reserved(0), bCBWCBLength
    0x12, 0, 0, 0,          // byte 15-30: CBWCommandBlock
    0x24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static void transfer_cb2(usb_transfer_t *transfer) {

  printf("MJB Transfer 2 status %d, actual number of bytes transferred %d\n",
         transfer->status, transfer->actual_num_bytes);
}

static void transfer_cb(usb_transfer_t *transfer) {

  printf("MJB Transfer status %d, actual number of bytes transferred %d\n",
         transfer->status, transfer->actual_num_bytes);

  class_driver_t *driver_obj = (class_driver_t  *)transfer->context;

  usb_transfer_t *transferR;
  usb_host_transfer_alloc(1024, 0, &transferR);
  
//  printf ("transferR = %lx\n",(unsigned long)transferR);

  // Perform an IN transfer from EP1
  transferR->num_bytes = 64;
  transferR->device_handle = driver_obj->dev_hdl;
  transferR->bEndpointAddress = 0x81;
  transferR->callback = transfer_cb2;
  transferR->context = (void *)driver_obj;
  esp_err_t r = usb_host_transfer_submit(transferR);

  printf("MJB usb_host_transfer_submit () result %s\n", esp_err_to_name(r));  
  
}  

static void action_mytest(class_driver_t *driver_obj) {

  usb_transfer_t *transfer;
  usb_host_transfer_alloc(1024, 0, &transfer);

  assert(driver_obj->dev_hdl != NULL);

  printf("MJB action_mytest()\n");

  esp_err_t r = usb_host_interface_claim(driver_obj->client_hdl,
                                         driver_obj->dev_hdl, 0, 0);

  printf("MJB usb_host_interface_claim() result %s\n", esp_err_to_name(r));

  // Send an OUT transfer to EP2
   memcpy(transfer->data_buffer,requestSenseCmd, 31);
//  memcpy(transfer->data_buffer, inquiryCmd, 31);
  transfer->num_bytes = 31;
  transfer->device_handle = driver_obj->dev_hdl;
  transfer->bEndpointAddress = 0x02;
  transfer->callback = transfer_cb;
  transfer->context = (void *)driver_obj;
  r = usb_host_transfer_submit(transfer);

  printf("MJB usb_host_transfer_submit () result %s\n", esp_err_to_name(r));

  driver_obj->actions &= ~ACTION_MYTEST;
}


static void action_close_dev(class_driver_t *driver_obj)
{
    ESP_ERROR_CHECK(usb_host_device_close(driver_obj->client_hdl, driver_obj->dev_hdl));
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    //We need to exit the event handler loop
    driver_obj->actions &= ~ACTION_CLOSE_DEV;
    driver_obj->actions |= ACTION_EXIT;
}

void class_driver_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
    class_driver_t driver_obj = {0};

    //Wait until daemon task has installed USB Host Library
    xSemaphoreTake(signaling_sem, portMAX_DELAY);

    ESP_LOGI(TAG, "Registering Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,    //Synchronous clients currently not supported. Set this to false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *)&driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj.client_hdl));

    while (1) {
        if (driver_obj.actions == 0) {
            usb_host_client_handle_events(driver_obj.client_hdl, portMAX_DELAY);
        } else {
            if (driver_obj.actions & ACTION_OPEN_DEV) {
                action_open_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_INFO) {
                action_get_info(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_DESC) {
                action_get_dev_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_CONFIG_DESC) {
                action_get_config_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_STR_DESC) {
                action_get_str_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_CLOSE_DEV) {
                action_close_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_EXIT) {
                break;
            }
            if (driver_obj.actions & ACTION_MYTEST) {
                action_mytest(&driver_obj);
            }            
        }
    }

    ESP_LOGI(TAG, "Deregistering Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(driver_obj.client_hdl));

    //Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}
