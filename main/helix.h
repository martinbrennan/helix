#include <stdint.h>


#define THREADSTACKSIZE 4096


typedef uint16_t u16;
typedef int16_t s16;
typedef uint8_t u8;

// class_driver.c

int testUnitReady ();
void requestSense ();
void readBlocks (int sector, int count);
void readToc ();
int waitForCommand (int seconds);

// lcd.c

void lcdInit ();

// audio.c

void audioInit(void);
void startAudioThread();

// utils.c

typedef struct {

	pthread_mutex_t mutex;			// lock to stop threads interfering
	pthread_cond_t condition;		// so producer can wait for consumer
	u16 *buffer;					// base of sample buffer
	int bufferSize;					// size of buffer in frames (pair of u16 samples)
	u16 *top;						// top of buffer
	u16 *in;						// address for putting samples into buffer
	u16 *out;						// address for taking samples from buffer
	int occupancy;					// number of frames in buffer
	int underrun;					// buffer has underrun
	int overrun;					// buffer has overrun
	int escape;						// allows the producer to be terminated if consumer has stopped

} smartControl;

smartControl * createSmartBuffer (long size);
