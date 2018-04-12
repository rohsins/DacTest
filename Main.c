#include <LPC17xx.h>
#include <UART_LPC17xx.h>
#include <GPIO_LPC17xx.h>
#include <cmsis_os.h>
#include "ITM_ARM.h"
#include <string.h>

#define IER_RBR 1U << 0
#define IER_THRE 1U << 1
#define IER_RLS 1U << 2

#define	DutyCycle0 LPC_PWM1->MR1
#define DutyCycle1	LPC_PWM1->MR2
#define DutyCycle2 LPC_PWM1->MR3
#define DutyCycle3	LPC_PWM1->MR4
#define DutyCycle4 LPC_PWM1->MR5

extern ARM_DRIVER_USART Driver_USART0;
extern ARM_DRIVER_USART Driver_USART1;

ARM_USART_STATUS Driver_USART1_STATUS;
USART_TRANSFER_INFO Driver_USART1_INFO;

char tempwhat[32];

void USART_callback(uint32_t event)
{
//		static int i;
	
    switch (event)
    {
    case ARM_USART_EVENT_RECEIVE_COMPLETE: 
//				itmPrintln("receive complete");
//					ringBufferWrite(pData);
//					ringBufferRead(readout);
//					itmPrint("readout:"); itmPrintlnInt(*readout);
//					i++;
//				itmPrintln(temp);
				break;
    case ARM_USART_EVENT_TRANSFER_COMPLETE:
//			itmPrintln("transfer complete");
			break;
    case ARM_USART_EVENT_SEND_COMPLETE:
//			itmPrintln("send complete");
			break;
    case ARM_USART_EVENT_TX_COMPLETE:
//			itmPrintln("tx complete");
        /* Success: Wakeup Thread */
//        osSignalSet(tid_myUART_Thread, 0x01);
        break;
 
    case ARM_USART_EVENT_RX_TIMEOUT:
//			itmPrintln("rx timeout");
         __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
        break;
 
    case ARM_USART_EVENT_RX_OVERFLOW:
//			itmPrintln("rx overflow");
			break;
    case ARM_USART_EVENT_TX_UNDERFLOW:
//        __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
		break;		
	}
}

void uart0Initialize(void) {
	Driver_USART0.Initialize(USART_callback);
	Driver_USART0.PowerControl(ARM_POWER_FULL);
	Driver_USART0.Control(ARM_USART_MODE_ASYNCHRONOUS| ARM_USART_DATA_BITS_8| ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE,115200);
	Driver_USART0.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART0.Control(ARM_USART_CONTROL_RX,1);      

	NVIC_EnableIRQ(UART0_IRQn);
	LPC_UART0->IER = IER_RBR | IER_THRE |IER_RLS; //Enable Interrupt
}

void uart0UnInitialize(void) {
	Driver_USART0.Uninitialize();
}

void uart1Initialize(void) {
	Driver_USART1.Initialize(USART_callback);
	Driver_USART1.PowerControl(ARM_POWER_FULL);
	Driver_USART1.Control(ARM_USART_MODE_ASYNCHRONOUS| ARM_USART_DATA_BITS_8| ARM_USART_PARITY_NONE | ARM_USART_STOP_BITS_1 | ARM_USART_FLOW_CONTROL_NONE,115200);
	Driver_USART1.Control(ARM_USART_CONTROL_TX,1);
	Driver_USART1.Control(ARM_USART_CONTROL_RX,1);  

	NVIC_EnableIRQ(UART1_IRQn);
	LPC_UART1->IER |= IER_RBR | IER_THRE |IER_RLS; //Enable Interrupt
}

void uart1UnInitialize(void) {
	Driver_USART1.Uninitialize();
}

void ledInitialize(void) {
	GPIO_SetDir(2, 7, GPIO_DIR_OUTPUT);
}                                                                                                                                      

void configPWM(void) {
	LPC_PINCON->PINSEL4 = (1<<0) | (1<<2) | (1<<4) | (1<<6) | (1<<8); //PWM ON PIN 0, 1, 2, 3, 4
	LPC_PWM1->TCR = (1<<0) | (1<<2);
	LPC_PWM1->PR = 0x10;
	LPC_PWM1->MCR = (1<<1);
	
	LPC_PWM1->MR0 = 100; //set period to 100%
	
	DutyCycle0 = 80;
	DutyCycle1 = 80;
	DutyCycle2 = 80;
	DutyCycle3 = 80;
	DutyCycle4 = 80;
	
	LPC_PWM1->LER = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5); //PWM ON PIN 0, 1, 2, 3, 4
	
	LPC_PWM1->PCR = (1<<9) | (1<<10) | (1<<11) | (1<<12) | (1<<13); //PWM ON PIN 0, 1, 2, 3, 4
}

void pwmRunner(void const * params) {
	int dutyCycle = 0;
	
	configPWM();
	
	while(1)
    {
        for(dutyCycle=1; dutyCycle<100; dutyCycle++)
        {
            DutyCycle0 = dutyCycle;
            DutyCycle1 = dutyCycle;
            DutyCycle2 = dutyCycle;
            DutyCycle3 = dutyCycle;
						DutyCycle4 = dutyCycle;
            osDelay(20);
        }

        for(dutyCycle=99; dutyCycle>0; dutyCycle--)
        {
            DutyCycle0 = dutyCycle;
            DutyCycle1 = dutyCycle;
            DutyCycle2 = dutyCycle;
            DutyCycle3 = dutyCycle;
						DutyCycle4 = dutyCycle;
            osDelay(20);
        }
    }
}
osThreadDef(pwmRunner, osPriorityNormal, 1, 0);
	
void heartBeatThread(void const *arg) {
	while (1) {
		GPIO_PinWrite(2, 7, 0);
		osDelay(70);
		GPIO_PinWrite(2, 7, 1);
		osDelay(1000);
	}
}
osThreadDef(heartBeatThread, osPriorityNormal, 1, 0);

int main(void) {
	
	SystemCoreClockUpdate ();
	SysTick_Config(SystemCoreClock/1000);
		
	osKernelInitialize();

	uart0Initialize();
	uart1Initialize();
	osDelay(100);
	ledInitialize();
	
	osThreadCreate(osThread(heartBeatThread), NULL);
	osThreadCreate(osThread(pwmRunner), NULL);
	osKernelStart();
	
	return 0;
}
