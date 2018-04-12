#include "ITM_ARM.h"
#include <stdlib.h>

#ifdef __cplusplus
	extern "C"
#endif


void itmPrintln(volatile char *string) {
	while(*string) {
		while (ITM_Port8(0) == 0);
		ITM_Port8(0) = *string++;     /* displays value in ASCII */
	}
	while (ITM_Port8(0) == 0);
	ITM_Port8(0) = 0x0D;
	while (ITM_Port8(0) == 0);
	ITM_Port8(0) = 0x0A;
}

void itmPrint(volatile char *string) {
	while(*string) {
		while (ITM_Port8(0) == 0);
		ITM_Port8(0) = *string++;     /* displays value in ASCII */
	}
}

void itmPrintlnInt(volatile int integer) {
	volatile char temp[32];
	sprintf(temp, "%d", integer);
	itmPrintln(temp);
}

void itmPrintInt(volatile int integer) {
	volatile char temp[32];
	sprintf(temp, "%d", integer);
	itmPrint(temp);
}
