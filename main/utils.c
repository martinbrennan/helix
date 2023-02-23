#include <pthread.h>
#include <string.h>
#include "helix.h"


int mymin (int a, int b){
	return a < b ? a : b;
}

// The smart buffer utilities are from play.c in the B2 project

void clearSmartBuffer (smartControl *sc){

	pthread_mutex_lock (&(sc->mutex));
	sc->in = sc->buffer;
	sc->out = sc->buffer;
	sc->occupancy = 0;
	sc->underrun = 1;
	sc->overrun = 0;
	sc->escape = 0;
	pthread_mutex_unlock (&(sc->mutex));
}

smartControl * createSmartBuffer (long size){

	smartControl *control;
	u8 *b;
	
	control = (smartControl *) malloc (sizeof (smartControl));
	b = (u8 *) malloc (size * 4);
	
	pthread_mutex_init (&(control->mutex),NULL);
	pthread_cond_init (&(control->condition),NULL);
	control->buffer = (u16 *)b;
	control->bufferSize = size;
	control->top = (u16 *)(b + (size * 4));
	clearSmartBuffer (control);

	return control;
}

// This variant waits for enough space


void smartPutSamplesW (smartControl *sc, int frames, s16 *source){

	int b;													// frames to move in [each] memmove

	pthread_mutex_lock(&(sc->mutex));

//	printf ("Put occupancy %d escape %d\n",sc->occupancy,sc->escape);

	while (((sc->occupancy + frames) > sc->bufferSize) && (!sc->escape)) {
//		printf ("smartPutSamplesW () wait\n");
		pthread_cond_wait(&(sc->condition), &(sc->mutex));
	}
	
	if (sc->escape){
		sc->escape = 0;
		pthread_mutex_unlock (&(sc->mutex));
		return;
	}
	
	sc->occupancy += frames;
	while (frames){
		b = mymin (frames,(sc->top - sc->in)/2);				// number of frames to top of buffer
		memmove (sc->in,source,b*4);
		source += b*2;
		sc->in += b*2;
		if (sc->in >= sc->top) sc->in = sc->buffer;
		frames -= b;
	}
	
	pthread_mutex_unlock (&(sc->mutex));	
}

// This should be called so that producer thread can be terminated

void releasePendingPutSamples (smartControl *sc){

	sc->escape = 1;
	pthread_cond_signal (&(sc->condition));	
	
}

// This variant signals the condition variable so that producer can wait
// returns 0 if not enough yet

int smartGetSamplesS (smartControl *sc, int frames, s16 *dest){

	int b;													// frames to move in [each] memmove

	if (sc->occupancy < frames) return 0;

	pthread_mutex_lock (&(sc->mutex));	
	
	sc->occupancy -= frames;	
	while (frames){
		b = mymin (frames,(sc->top - sc->out)/2);				// number of frames to top of buffer
		memmove (dest,sc->out,b*4);
		dest += b*2;
		sc->out += b*2;
		if (sc->out >= sc->top) sc->out = sc->buffer;
		frames -= b;
	}

	pthread_cond_signal (&(sc->condition));					// signal that buffer may have changed	
	pthread_mutex_unlock (&(sc->mutex));

	return 1;
}
