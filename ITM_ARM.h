#ifndef _ITM_ARM_H_
#define _ITM_ARM_H_

#define ITM_Port8(n)   (*((volatile unsigned char *)(0xE0000000+4*n)))
	
void itmPrintln(char *string);
void itmPrint(char *string);
void itmPrintlnInt(int integer);
void itmPrintInt(int integer);
	
#endif
