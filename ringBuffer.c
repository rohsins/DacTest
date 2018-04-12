#include <stdint.h>

#define BUFSIZE 100
#define RINGBUFFSIZE 100
#define RINGBUFFLENGTH (RINGBUFFSIZE + 1)

char temp[100];
char *readout = temp;

char ringBuffer[RINGBUFFLENGTH];
unsigned int head, tail;

void ringBufferInit(void) {
	head = tail = 0;
}

uint32_t ringBufferWrite(char in) {
	if (head == ((tail - 1 + RINGBUFFLENGTH) % 	RINGBUFFLENGTH)) {
		return 1;
	}
	
	ringBuffer[head] = in;
	head = (head + 1) % RINGBUFFLENGTH;
	return 0;
}

uint32_t ringBufferRead(char *out) {
	if (head == tail) {
		return 1;
	}
	*out = ringBuffer[tail];
//	ringBuffer[tail] = 0; // just for acknowledge;
	tail = ((tail + 1) % RINGBUFFLENGTH);
	return 0; 
}