
#define THREADSTACKSIZE 4096


// class_driver.c

void testUnitReady ();
void requestSense ();
void readBlocks (int sector, int count);
void readToc ();


// lcd.c

void lcdInit ();

// audio.c

void audioInit(void);
void startAudioThread();
