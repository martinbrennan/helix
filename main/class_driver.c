
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "helix.h"
#include "usb/usb_host.h"
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"

#include <pthread.h>
#include <time.h>

pthread_mutex_t commandMutex;
pthread_cond_t commandCond;

// returns nz on timeout

int waitForCommand (int seconds){

	int r;
	struct timespec wait_time;
	clock_gettime (CLOCK_REALTIME, &wait_time);
	wait_time.tv_sec += seconds;
	
	pthread_mutex_lock (&commandMutex);
	r = pthread_cond_timedwait (&commandCond, &commandMutex, &wait_time);
	pthread_mutex_unlock (&commandMutex);
	if (r) printf ("waitForCommand Timeout\n");	
	return r;	
}	

void commandDone (){
	pthread_mutex_lock (&commandMutex);
	pthread_cond_signal (&commandCond);
	pthread_mutex_unlock (&commandMutex);	
}

#define CLIENT_NUM_EVENT_MSG 5

#define ACTION_OPEN_DEV 0x01
#define ACTION_GET_DEV_INFO 0x02
#define ACTION_GET_DEV_DESC 0x04
#define ACTION_GET_CONFIG_DESC 0x08
#define ACTION_GET_STR_DESC 0x10
#define ACTION_CLOSE_DEV 0x20
#define ACTION_EXIT 0x40

#define ACTION_MYTEST 0x80

int testUnitReadyFlag = 0;
int readFlag = 0;
int readTocFlag = 0;
int ejectFlag = 0;
int loadFlag = 0;
int requestSenseFlag = 0;
usb_host_client_handle_t publicHandle;

int tocTrack = 0;
int tocCount = 1;
int trackCount = 0;
int trackStart[100];

int getTrackStart (int track){
	return trackStart[track];
}

int getTrackCount (){
	return trackCount;
}		

usb_transfer_t *commandTransfer;
usb_transfer_t *dataTransfer;
usb_transfer_t *statusTransfer;



typedef struct {
  usb_host_client_handle_t client_hdl;
  uint8_t dev_addr;
  usb_device_handle_t dev_hdl;
  uint32_t actions;
} class_driver_t;

static const char *TAG = "CLASS";

static void client_event_cb(const usb_host_client_event_msg_t *event_msg,
                            void *arg) {
  class_driver_t *driver_obj = (class_driver_t *)arg;
  switch (event_msg->event) {
  case USB_HOST_CLIENT_EVENT_NEW_DEV:
    if (driver_obj->dev_addr == 0) {
      driver_obj->dev_addr = event_msg->new_dev.address;
      // Open the device next
      driver_obj->actions |= ACTION_OPEN_DEV;
    }
    break;
  case USB_HOST_CLIENT_EVENT_DEV_GONE:
    if (driver_obj->dev_hdl != NULL) {
      // Cancel any other actions and close the device next
      driver_obj->actions = ACTION_CLOSE_DEV;
    }
    break;
  default:
    // Should never occur
    abort();
  }
}

static void action_open_dev(class_driver_t *driver_obj) {
  assert(driver_obj->dev_addr != 0);
  ESP_LOGI(TAG, "Opening device at address %d", driver_obj->dev_addr);
  ESP_ERROR_CHECK(usb_host_device_open(
      driver_obj->client_hdl, driver_obj->dev_addr, &driver_obj->dev_hdl));
  // Get the device's information next
  driver_obj->actions &= ~ACTION_OPEN_DEV;
  driver_obj->actions |= ACTION_GET_DEV_INFO;
}

static void action_get_info(class_driver_t *driver_obj) {
  assert(driver_obj->dev_hdl != NULL);
  ESP_LOGI(TAG, "Getting device information");
  usb_device_info_t dev_info;
  ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
  ESP_LOGI(TAG, "\t%s speed",
           (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
  ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
  // Todo: Print string descriptors

  // Get the device descriptor next
  driver_obj->actions &= ~ACTION_GET_DEV_INFO;
  driver_obj->actions |= ACTION_GET_DEV_DESC;
}

static void action_get_dev_desc(class_driver_t *driver_obj) {
  assert(driver_obj->dev_hdl != NULL);
  ESP_LOGI(TAG, "Getting device descriptor");
  const usb_device_desc_t *dev_desc;
  ESP_ERROR_CHECK(
      usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc));
  usb_print_device_descriptor(dev_desc);
  // Get the device's config descriptor next
  driver_obj->actions &= ~ACTION_GET_DEV_DESC;
  driver_obj->actions |= ACTION_GET_CONFIG_DESC;
}

static void action_get_config_desc(class_driver_t *driver_obj) {
  assert(driver_obj->dev_hdl != NULL);
  ESP_LOGI(TAG, "Getting config descriptor");
  const usb_config_desc_t *config_desc;
  ESP_ERROR_CHECK(
      usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
  usb_print_config_descriptor(config_desc, NULL);
  // Get the device's string descriptors next
  driver_obj->actions &= ~ACTION_GET_CONFIG_DESC;
  driver_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(class_driver_t *driver_obj) {
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
  // Nothing to do until the device disconnects

  printf("MJB end of action_get_str_desc()\n");

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

unsigned char cBW[31] = {
    0x55, 0x53, 0x42, 0x43, // byte 0-3: dCBWSignature
    0,    0,    0,    1,    // byte 4-7: dCBWTag
    0,    0,    0,    0,    // byte 8-11: dCBWDataTransferLength
    0x80,                   // byte 12: bmCBWFlags
    0,                      // byte 13: bit 7-4 Reserved(0), bCBWLUN
    0x0C,                   // byte 14: bit 7-5 Reserved(0), bCBWCBLength
    0xBE, 0,    0,    0,    // byte 15-30: CBWCommandBlock
    0,    0,    0,    0,    0, 0xF0, 0, 0, 0, 0, 0, 0};

int lba = 75 * 60;
int blocks = 10;

int setupRead() {

  int count = BLOCKSIZE;
  int datalength = count * 2352;

  cBW[8] = (datalength & 0xff);
  cBW[9] = ((datalength >> 8) & 0xff);
  cBW[10] = ((datalength >> 16) & 0xff);
  cBW[11] = ((datalength >> 24) & 0xff);

  cBW[20] = (lba & 0xff);
  cBW[19] = ((lba >> 8) & 0xff);
  cBW[18] = ((lba >> 16) & 0xff);
  cBW[17] = ((lba >> 24) & 0xff);

  cBW[23] = (count & 0xff);
  cBW[22] = ((count >> 8) & 0xff);
  cBW[21] = ((count >> 16) & 0xff);

  lba += count;

  return (datalength + 63) & 0xFFFFFFC0;
}







void cdump (unsigned char *buf, int len){
	
	int n = 1;
	while (len--){
		printf ("%02x ",*buf++);
		if (!(n++ % 8)) printf ("\n");
	}
	if ((n - 1) % 8) printf ("\n");	
}		

int testUnitReadyResult = 0;

static void testUnitReadycb2(usb_transfer_t *transfer) {
/*
  printf("testUnitReadycb2 status %d, actual number of bytes transferred %d\n",
         transfer->status, transfer->actual_num_bytes);
         
  cdump ((unsigned char *)transfer->data_buffer,transfer->actual_num_bytes);
*/
	unsigned char *r = (unsigned char *)transfer->data_buffer;

	if (r[12]) printf ("Test Unit Ready Failed\n");
	else printf ("Test Unit Ready OK\n");

	testUnitReadyResult = !r[12];
	
	commandDone ();

}

void transfer_cb2(usb_transfer_t *transfer) {

  printf("transfer_cb2 status %d, actual number of bytes transferred %d\n",
         transfer->status, transfer->actual_num_bytes);
         
}



void transfer_cb(usb_transfer_t *transfer) {

  printf("MJB Transfer status %d, actual number of bytes transferred %d\n",
         transfer->status, transfer->actual_num_bytes);

  class_driver_t *driver_obj = (class_driver_t *)transfer->context;

  usb_transfer_t *transferR;
  usb_host_transfer_alloc(4096, 0, &transferR);

  //  printf ("transferR = %lx\n",(unsigned long)transferR);

  // Perform an IN transfer from EP1
  transferR->num_bytes = 64;
  //  transferR->num_bytes = (2352+63)&0xFFFFFFC0;
  transferR->device_handle = driver_obj->dev_hdl;
  transferR->bEndpointAddress = 0x81;
  transferR->callback = transfer_cb2;
  transferR->context = (void *)driver_obj;
  esp_err_t r = usb_host_transfer_submit(transferR);

  printf("MJB usb_host_transfer_submit 2 () result %s\n", esp_err_to_name(r));
}

// WARNING not a test - a necessary part of initiialisation

static void action_mytest(class_driver_t *driver_obj) {

  usb_transfer_t *transfer;
  usb_host_transfer_alloc(1024, 0, &transfer);

  assert(driver_obj->dev_hdl != NULL);

  printf("MJB action_mytest()\n");

  esp_err_t r = usb_host_interface_claim(driver_obj->client_hdl,
                                         driver_obj->dev_hdl, 0, 0);

  printf("MJB usb_host_interface_claim() result %s\n", esp_err_to_name(r));

  /*
    // Send an OUT transfer to EP2
    memcpy(transfer->data_buffer,requestSenseCmd, 31);		// gets 18 bytes
  back
  //  memcpy(transfer->data_buffer, inquiryCmd, 31);		// gets 36 bytes
  back
  //  memcpy(transfer->data_buffer, tocCmd, 31);		// gets
  //  int l = setupRead();
  //  memcpy(transfer->data_buffer, cBW, 31);
    transfer->num_bytes = 31;
    transfer->device_handle = driver_obj->dev_hdl;
    transfer->bEndpointAddress = 0x02;
    transfer->callback = transfer_cb;
    transfer->context = (void *)driver_obj;
    r = usb_host_transfer_submit(transfer);
    printf("MJB usb_host_transfer_submit () result %s\n", esp_err_to_name(r));
  */
  driver_obj->actions &= ~ACTION_MYTEST;
}

void requestSense() {
  printf("requestSense () handle %lx\n", (unsigned long)publicHandle);
  requestSenseFlag = 1;
  usb_host_client_unblock(publicHandle);
}

void readBlocks (int sector, int count){
	lba = sector;
	blocks = count;
  readFlag = 1;
  usb_host_client_unblock(publicHandle);
  waitForCommand (5);	
}

int readToc (int track, int count){
	tocTrack = track;
	tocCount = count;
  readTocFlag = 1;
  usb_host_client_unblock(publicHandle);
  waitForCommand (5);
  return trackCount;   	
}


void readcb3 (usb_transfer_t *transfer) {

if (transfer->actual_num_bytes != 13){
    printf("readb3 %d bytes\n", transfer->actual_num_bytes);
    cdump((unsigned char *)transfer->data_buffer, 8);  
}
  blocks--;  
  if (blocks) {
    readFlag = 1;
  } else {
    printf("All done\n");    
    commandDone ();
  }  
}

unsigned char *getDataAddress (){
	return (unsigned char *) dataTransfer->data_buffer;
}
	
static void readcb2 (usb_transfer_t *transfer) {


    printf("readcb2 %d bytes from lba %d\n", transfer->actual_num_bytes,lba);
//    cdump((unsigned char *)transfer->data_buffer, 8);

// get status

  class_driver_t *driver_obj = (class_driver_t *)transfer->context;
    
  statusTransfer->num_bytes = 64;			// rounded up
  statusTransfer->device_handle = driver_obj->dev_hdl;
  statusTransfer->bEndpointAddress = 0x81;
  statusTransfer->callback = readcb3;
  statusTransfer->context = (void *)driver_obj;
  esp_err_t r = usb_host_transfer_submit(statusTransfer);

  if (r != ESP_OK)
    printf("readcb2 submit error %s\n", esp_err_to_name(r)); 

}


void readcb(usb_transfer_t *transfer) {
/*
  printf("MJB requestSensecb Transfer status %d, actual number of bytes "
         "transferred %d\n",
         transfer->status, transfer->actual_num_bytes);
*/
  class_driver_t *driver_obj = (class_driver_t *)transfer->context;

//  printf("One block rounded up %d\n", (2352 + 63) & 0xFFFFFFC0);	// 2368


  dataTransfer->num_bytes = 2352*BLOCKSIZE;		// 
  											
  dataTransfer->device_handle = driver_obj->dev_hdl;
  dataTransfer->bEndpointAddress = 0x81;
  dataTransfer->callback = readcb2;
  dataTransfer->context = (void *)driver_obj;
  esp_err_t r = usb_host_transfer_submit(dataTransfer);

  if (r != ESP_OK)
    printf("readcb submit error %s\n", esp_err_to_name(r));

}

void startRead(class_driver_t *driver_obj) {

  int r;

  assert(driver_obj->dev_hdl != NULL);

//  printf("startRead()\n");

  setupRead();
  memcpy(commandTransfer->data_buffer, cBW, 31);

  commandTransfer->num_bytes = 31;
  commandTransfer->device_handle = driver_obj->dev_hdl;
  commandTransfer->bEndpointAddress = 0x02;
  commandTransfer->callback = readcb;
  commandTransfer->context = (void *)driver_obj;
  r = usb_host_transfer_submit(commandTransfer);

  if (r != ESP_OK)
    printf("startRead submit error %s\n", esp_err_to_name(r));
}


static void readToccb3 (usb_transfer_t *transfer) {

//    printf("readToccb3 %d bytes\n", transfer->actual_num_bytes);
//    cdump((unsigned char *)transfer->data_buffer, 8);
	commandDone ();
}

// WARNING when a command goes wrong it sends 20 bytes status here
// at that point I should not request the status

static void readToccb2 (usb_transfer_t *transfer) {

//    printf("readToccb2 %d bytes\n", transfer->actual_num_bytes);
//    cdump((unsigned char *)transfer->data_buffer,transfer->actual_num_bytes);  

	unsigned char *d = transfer->data_buffer;
	trackCount = d[3];
    
		int track;
		for (track = 0;track < tocCount;track++){
			int i = (track << 3) + 4;
			int lba = d[i+7];
			lba += d[i+6]<<8;
			lba += d[i+5]<<16;
			lba += d[i+4]<<24;
//			printf ("got lba %x\n",lba);
			trackStart[track] = lba;
		}	
    
// get status

  class_driver_t *driver_obj = (class_driver_t *)transfer->context;
    
  dataTransfer->num_bytes = 64;			// rounded up
  dataTransfer->device_handle = driver_obj->dev_hdl;
  dataTransfer->bEndpointAddress = 0x81;
  dataTransfer->callback = readToccb3;
  dataTransfer->context = (void *)driver_obj;
  esp_err_t r = usb_host_transfer_submit(dataTransfer);

  if (r != ESP_OK)
    printf("readToccb2 submit error %s\n", esp_err_to_name(r));    
    
}

void readToccb(usb_transfer_t *transfer) {
/*
  printf("readToccb Transfer status %d, actual number of bytes "
         "transferred %d\n",
         transfer->status, transfer->actual_num_bytes);
*/
  class_driver_t *driver_obj = (class_driver_t *)transfer->context;

//  dataTransfer->num_bytes = 804;
//  dataTransfer->num_bytes = 832;			// rounded up

  int byteCount = tocCount * 8 + 4;
  byteCount += 63;						// round up
  byteCount |= 0x3F;
  byteCount ^= 0x3F;

  dataTransfer->num_bytes = byteCount;			// rounded up
  dataTransfer->device_handle = driver_obj->dev_hdl;
  dataTransfer->bEndpointAddress = 0x81;
  dataTransfer->callback = readToccb2;
  dataTransfer->context = (void *)driver_obj;
  esp_err_t r = usb_host_transfer_submit(dataTransfer);

  if (r != ESP_OK)
    printf("readToccb submit error %s\n", esp_err_to_name(r));

}

unsigned char tocCmd[31] = {0x55, 0x53, 0x42, 0x43, // byte 0-3:
                                                    // dCBWSignature
                            0, 0, 0, 1,             // byte 4-7: dCBWTag
                            0x24, 3, 0, 0, // byte 8-11: dCBWDataTransferLength
                            0x80,          // byte 12: bmCBWFlags
                            0,    // byte 13: bit 7-4 Reserved(0), bCBWLUN
                            0x0A, // byte 14: bit 7-5 Reserved(0), bCBWCBLength
                            0x43, 0, 0, 0, // byte 15-30: CBWCommandBlock
											// bit 1 of byte 16 is MSF flag
                            0, 0, 0, 3, 0x24, 0, 0, 0, 0, 0, 0, 0};


void startReadToc (class_driver_t *driver_obj) {

  int r;

  assert(driver_obj->dev_hdl != NULL);

//  printf("startReadToc ()\n");

	int datalength = tocCount * 8 + 4;

  tocCmd[8] = (datalength & 0xff);
  tocCmd[9] = ((datalength >> 8) & 0xff);
  tocCmd[10] = ((datalength >> 16) & 0xff);
  tocCmd[11] = ((datalength >> 24) & 0xff);

  tocCmd[23] = (datalength & 0xff);
  tocCmd[22] = ((datalength >> 8) & 0xff);

  tocCmd[21] = tocTrack;

	memcpy(commandTransfer->data_buffer,tocCmd, 31);
//	memcpy(commandTransfer->data_buffer,tocCmd3, 31);

  commandTransfer->num_bytes = 31;
  commandTransfer->device_handle = driver_obj->dev_hdl;
  commandTransfer->bEndpointAddress = 0x02;
  commandTransfer->callback = readToccb;
  commandTransfer->context = (void *)driver_obj;
  r = usb_host_transfer_submit(commandTransfer);

  if (r != ESP_OK)
    printf("startReadToc submit error %s\n", esp_err_to_name(r));
}

static void ejectCb2(usb_transfer_t *transfer) {

  printf("Ejectcb2 status %d, actual number of bytes transferred %d\n",
         transfer->status, transfer->actual_num_bytes);

  cdump((unsigned char *)transfer->data_buffer, transfer->actual_num_bytes);

  commandDone();
}

unsigned char ejectCmd[31] = {
    0x55, 0x53, 0x42, 0x43, // byte 0-3:
                            // dCBWSignature
    0, 0, 0, 1,             // byte 4-7: dCBWTag
    0, 0, 0, 0,             // byte 8-11: dCBWDataTransferLength
    0,                      // byte 12: bmCBWFlags
    0,                      // byte 13: bit 7-4 Reserved(0), bCBWLUN
    0x06,                   // byte 14: bit 7-5 Reserved(0), bCBWCBLength
    0x1B, 0, 0, 0,             // byte 15-30: CBWCommandBlock
    2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int eject() {
//  printf("eject ()\n");
  ejectFlag = 1;
  usb_host_client_unblock(publicHandle);
  waitForCommand (10);
  return 1;  
}

void ejectCb(usb_transfer_t *transfer) {

  class_driver_t *driver_obj = (class_driver_t *)transfer->context;

  // Perform an IN transfer from EP1
  dataTransfer->num_bytes = 64;
  dataTransfer->device_handle = driver_obj->dev_hdl;
  dataTransfer->bEndpointAddress = 0x81;
  dataTransfer->callback = ejectCb2;
  dataTransfer->context = (void *)driver_obj;
  esp_err_t r = usb_host_transfer_submit(dataTransfer);

  if (r != ESP_OK)
    printf("ejectCb submit error %s\n", esp_err_to_name(r));
}

void startEject(class_driver_t *driver_obj) {

  int r;

  assert(driver_obj->dev_hdl != NULL);

  memcpy(commandTransfer->data_buffer, ejectCmd, 31);

  commandTransfer->num_bytes = 31;
  commandTransfer->device_handle = driver_obj->dev_hdl;
  commandTransfer->bEndpointAddress = 0x02;
  commandTransfer->callback = ejectCb;
  commandTransfer->context = (void *)driver_obj;
  r = usb_host_transfer_submit(commandTransfer);

  if (r != ESP_OK)
    printf("startEject submit error %s\n", esp_err_to_name(r));
}

static void loadCb2(usb_transfer_t *transfer) {

  printf("loadcb2 status %d, actual number of bytes transferred %d\n",
         transfer->status, transfer->actual_num_bytes);

  cdump((unsigned char *)transfer->data_buffer, transfer->actual_num_bytes);

  commandDone();
}

unsigned char loadCmd[31] = {
    0x55, 0x53, 0x42, 0x43, // byte 0-3:
                            // dCBWSignature
    0, 0, 0, 1,             // byte 4-7: dCBWTag
    0, 0, 0, 0,             // byte 8-11: dCBWDataTransferLength
    0,                      // byte 12: bmCBWFlags
    0,                      // byte 13: bit 7-4 Reserved(0), bCBWLUN
    0x06,                   // byte 14: bit 7-5 Reserved(0), bCBWCBLength
    0x1B, 0, 0, 0,             // byte 15-30: CBWCommandBlock
    3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int load() {
//  printf("eject ()\n");
  loadFlag = 1;
  usb_host_client_unblock(publicHandle);
  waitForCommand (5);
  return 1;  
}

void loadCb(usb_transfer_t *transfer) {

  class_driver_t *driver_obj = (class_driver_t *)transfer->context;

  // Perform an IN transfer from EP1
  dataTransfer->num_bytes = 64;
  dataTransfer->device_handle = driver_obj->dev_hdl;
  dataTransfer->bEndpointAddress = 0x81;
  dataTransfer->callback = loadCb2;
  dataTransfer->context = (void *)driver_obj;
  esp_err_t r = usb_host_transfer_submit(dataTransfer);

  if (r != ESP_OK)
    printf("loadCb submit error %s\n", esp_err_to_name(r));
}

void startLoad(class_driver_t *driver_obj) {

  int r;

  assert(driver_obj->dev_hdl != NULL);

  memcpy(commandTransfer->data_buffer, loadCmd, 31);

  commandTransfer->num_bytes = 31;
  commandTransfer->device_handle = driver_obj->dev_hdl;
  commandTransfer->bEndpointAddress = 0x02;
  commandTransfer->callback = loadCb;
  commandTransfer->context = (void *)driver_obj;
  r = usb_host_transfer_submit(commandTransfer);

  if (r != ESP_OK)
    printf("startLoad submit error %s\n", esp_err_to_name(r));
}


unsigned char readyCmd[31] = {
    0x55, 0x53, 0x42, 0x43, // byte 0-3:
                            // dCBWSignature
    0, 0, 0, 1,             // byte 4-7: dCBWTag
    0, 0, 0, 0,             // byte 8-11: dCBWDataTransferLength
    0,                      // byte 12: bmCBWFlags
    0,                      // byte 13: bit 7-4 Reserved(0), bCBWLUN
    0x06,                   // byte 14: bit 7-5 Reserved(0), bCBWCBLength
    0, 0, 0, 0,             // byte 15-30: CBWCommandBlock
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int testUnitReady() {
//  printf("testUnitReady ()\n");
  testUnitReadyFlag = 1;
  usb_host_client_unblock(publicHandle);
  waitForCommand (5);
  return testUnitReadyResult;  
}

void testUnitReadycb(usb_transfer_t *transfer) {

  class_driver_t *driver_obj = (class_driver_t *)transfer->context;

  // Perform an IN transfer from EP1
  dataTransfer->num_bytes = 64;
  dataTransfer->device_handle = driver_obj->dev_hdl;
  dataTransfer->bEndpointAddress = 0x81;
  dataTransfer->callback = testUnitReadycb2;
  dataTransfer->context = (void *)driver_obj;
  esp_err_t r = usb_host_transfer_submit(dataTransfer);

  if (r != ESP_OK)
    printf("startTestUnitReadycb submit error %s\n", esp_err_to_name(r));
}

void startTestUnitReady(class_driver_t *driver_obj) {

  int r;

  assert(driver_obj->dev_hdl != NULL);

  memcpy(commandTransfer->data_buffer, readyCmd, 31);

  commandTransfer->num_bytes = 31;
  commandTransfer->device_handle = driver_obj->dev_hdl;
  commandTransfer->bEndpointAddress = 0x02;
  commandTransfer->callback = testUnitReadycb;
  commandTransfer->context = (void *)driver_obj;
  r = usb_host_transfer_submit(commandTransfer);

  if (r != ESP_OK)
    printf("startTestUnitReady submit error %s\n", esp_err_to_name(r));
}

static void requestSensecb2(usb_transfer_t *transfer) {

  printf("requestSensecb2 status %d, actual number of bytes transferred %d\n",
         transfer->status, transfer->actual_num_bytes);
         
  cdump ((unsigned char *)transfer->data_buffer,transfer->actual_num_bytes);

}

void requestSensecb(usb_transfer_t *transfer) {

  class_driver_t *driver_obj = (class_driver_t *)transfer->context;

  // Perform an IN transfer from EP1
  dataTransfer->num_bytes = 64;
  dataTransfer->device_handle = driver_obj->dev_hdl;
  dataTransfer->bEndpointAddress = 0x81;
  dataTransfer->callback = requestSensecb2;
  dataTransfer->context = (void *)driver_obj;
  esp_err_t r = usb_host_transfer_submit(dataTransfer);

  if (r != ESP_OK)
    printf("requestSensecb submit error %s\n", esp_err_to_name(r));
}

void startRequestSense(class_driver_t *driver_obj) {

  int r;

  assert(driver_obj->dev_hdl != NULL);

  memcpy(commandTransfer->data_buffer, requestSenseCmd, 31);

  commandTransfer->num_bytes = 31;
  commandTransfer->device_handle = driver_obj->dev_hdl;
  commandTransfer->bEndpointAddress = 0x02;
  commandTransfer->callback = requestSensecb;
  commandTransfer->context = (void *)driver_obj;
  r = usb_host_transfer_submit(commandTransfer);

  if (r != ESP_OK)
    printf("startTestUnitReady submit error %s\n", esp_err_to_name(r));
}



static void action_close_dev(class_driver_t *driver_obj) {
  ESP_ERROR_CHECK(
      usb_host_device_close(driver_obj->client_hdl, driver_obj->dev_hdl));
  driver_obj->dev_hdl = NULL;
  driver_obj->dev_addr = 0;
  // We need to exit the event handler loop
  driver_obj->actions &= ~ACTION_CLOSE_DEV;
  driver_obj->actions |= ACTION_EXIT;
}

void class_driver_task(void *arg) {
	
// initialise command wait mechanism

	pthread_mutex_init (&commandMutex,NULL);
	pthread_cond_init (&commandCond,NULL);
	
  SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;
  class_driver_t driver_obj = {0};

  // Wait until daemon task has installed USB Host Library
  xSemaphoreTake(signaling_sem, portMAX_DELAY);

  ESP_LOGI(TAG, "Registering Client");
  usb_host_client_config_t client_config = {
      .is_synchronous = false, // Synchronous clients currently not supported.
                               // Set this to false
      .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
      .async =
          {
              .client_event_callback = client_event_cb,
              .callback_arg = (void *)&driver_obj,
          },
  };
  ESP_ERROR_CHECK(
      usb_host_client_register(&client_config, &driver_obj.client_hdl));

  publicHandle = driver_obj.client_hdl;
  
  usb_host_transfer_alloc(1024, 0, &commandTransfer);  
//  usb_host_transfer_alloc(20480, 0, &dataTransfer);   
  usb_host_transfer_alloc(2352*BLOCKSIZE, 0, &dataTransfer); 
  usb_host_transfer_alloc(64, 0, &statusTransfer); 

  while (1) {
    if (testUnitReadyFlag) {
      testUnitReadyFlag = 0;
      startTestUnitReady(&driver_obj);
    }
    if (readFlag) {
      readFlag = 0;
      startRead(&driver_obj);
    }
    if (readTocFlag) {
     readTocFlag = 0;
      startReadToc(&driver_obj);
    }
    if (ejectFlag) {
     ejectFlag = 0;
      startEject(&driver_obj);
    }
    if (loadFlag) {
     loadFlag = 0;
      startLoad(&driver_obj);
    }
    if (requestSenseFlag) {
     requestSenseFlag = 0;
      startRequestSense(&driver_obj);
    }

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

  // Wait to be deleted
  xSemaphoreGive(signaling_sem);
  vTaskSuspend(NULL);
}

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

//extern void class_driver_task(void *arg);

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


SemaphoreHandle_t signaling_sem;
TaskHandle_t daemon_task_hdl;
TaskHandle_t class_driver_task_hdl;

void usbInit (){
	

		printf ("\n\n\nhostSet etc not needed on helix\n\n\n");
	
	hostSel();
	hostOn();
	vbusDevEn();	
	
    signaling_sem = xSemaphoreCreateBinary();





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

}

void waitForUSBToFinish (){

    //Wait for the tasks to complete
    for (int i = 0; i < 2; i++) {
        xSemaphoreTake(signaling_sem, portMAX_DELAY);
    }

    //Delete the tasks
    vTaskDelete(class_driver_task_hdl);
    vTaskDelete(daemon_task_hdl);
	
}


