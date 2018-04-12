#include <LPC17xx.h>
#include <cmsis_os2.h>
extern "C" {
	#include <GPIO_LPC17xx.h>
	#include <PIN_LPC17xx.h>
}
#include <UART_LPC17xx.h>
#include <I2C_LPC17xx.h>
#include <SPI_LPC17xx.h>
#include <stdio.h>
#include <stdlib.h>
#include "ITM_ARM.h"
#include <string>
#include "RingBuffer.h"
#include "Driver_SPI.h"

#define DutyCycle0 LPC_PWM1->MR1
#define DutyCycle1 LPC_PWM1->MR2
#define DutyCycle4 LPC_PWM1->MR3
#define DutyCycle2 LPC_PWM1->MR4
#define DutyCycle3 LPC_PWM1->MR5

const int IER_RBR = 1U << 0;
const int IER_THRE = 1U << 1;
const int IER_RLS = 1U << 2;
const int LPFTap = 64;

extern ARM_DRIVER_USART Driver_USART0;
extern ARM_DRIVER_USART Driver_USART1;
extern ARM_DRIVER_I2C Driver_I2C1;
extern ARM_DRIVER_SPI Driver_SPI1;

ARM_USART_STATUS Driver_USART1_STATUS;
USART_TRANSFER_INFO Driver_USART1_INFO;
osSemaphoreId_t semaphoreAdcId;
osSemaphoreAttr_t semaphoreAdcAttr;

char pData;
char readout[34];
static volatile uint16_t AD_last;     
static volatile bool AD_done;
static volatile uint16_t channel1;
static volatile uint16_t channel2;
static volatile uint16_t axisX;
static volatile uint16_t axisY;
RingBuffer ringBuffer;
using namespace std;

extern "C" {
	void ADC_IRQHandler(void) {
		volatile uint32_t adstat;
		adstat = LPC_ADC->ADSTAT;
		AD_last = (LPC_ADC->ADGDR >> 4) & 0xFFF;
		channel1 = (LPC_ADC->ADDR0 >> 4) & 0xFFFF;
		channel2 = (LPC_ADC->ADDR1 >> 4) & 0xFFFF;
		AD_done = true;
		osSemaphoreRelease(semaphoreAdcId);
	}
}

void USART_callback(uint32_t event) {
	switch (event) {
		case ARM_USART_EVENT_RECEIVE_COMPLETE: 
		//    readCheck = pData;
		//    ringBuffer.ringBufferWrite(pData);
		//    osSemaphoreRelease(semaphoreUartDataReadyId);
		//   itmPrintln("receive complete");
		break;
		case ARM_USART_EVENT_TRANSFER_COMPLETE:
		//    itmPrintln("transfer complete");
		break;
		case ARM_USART_EVENT_SEND_COMPLETE:
		//    itmPrintln("send complete");
		break;
		case ARM_USART_EVENT_TX_COMPLETE:
		//    itmPrintln("tx complete");
		/* Success: Wakeup Thread */
		//        osSignalSet(tid_myUART_Thread, 0x01);
		break;

		case ARM_USART_EVENT_RX_TIMEOUT:
		//    itmPrintln("rx timeout");
		__breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
		break;

		case ARM_USART_EVENT_RX_OVERFLOW:
		//    itmPrintln("rx overflow");
		break;
		case ARM_USART_EVENT_TX_UNDERFLOW:
		 __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
		break;  
	}
}

void SPI_callback(uint32_t event) {
 
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

void spi1Initialize(void) {
	Driver_SPI1.Initialize(SPI_callback);
	Driver_SPI1.PowerControl(ARM_POWER_FULL);
	Driver_SPI1.Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_SS_MASTER_SW | ARM_SPI_DATA_BITS(8), 1000000);
	Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}

void ledInitialize(void) {
	GPIO_SetDir(2, 7, GPIO_DIR_OUTPUT);
	GPIO_SetDir(2, 0, GPIO_DIR_OUTPUT);
	GPIO_SetDir(1, 30, GPIO_DIR_OUTPUT);
	GPIO_SetDir(1, 31, GPIO_DIR_OUTPUT);
	GPIO_PinWrite(1, 30, 1);
	GPIO_PinWrite(1, 31, 0);
	GPIO_PinWrite(2, 0, 0);
}  

void configPWM(void) {
	LPC_PINCON->PINSEL4 = (1<<0) | (1<<2) | (1<<4) | (1<<6) | (1<<8); //PWM ON PIN 0, 1, 2, 3, 4
	LPC_PWM1->TCR = (1<<0) | (1<<2);
	LPC_PWM1->PR = 0x18; // 0xF8 100Hz
	LPC_PWM1->MCR = (1<<1);

	LPC_PWM1->MR0 = 10000; //set period to 100%

	DutyCycle0 = 0;
	DutyCycle1 = 0;
	DutyCycle2 = 0;
	DutyCycle3 = 0;
	DutyCycle4 = 0;

	LPC_PWM1->LER = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5); //PWM ON PIN 0, 1, 2, 3, 4
	LPC_PWM1->PCR = (1<<9) | (1<<10) | (1<<11) | (1<<12) | (1<<13); //PWM ON PIN 0, 1, 2, 3, 4
}

const PIN ADC_PIN[] = {
	{0, 23}, {0, 24}
};

void ADC_Initialize (void) {
	LPC_SC->PCONP |= ((1 << 12) | (1 << 15));
	PIN_Configure (ADC_PIN[0].Portnum, ADC_PIN[0].Pinnum, PIN_FUNC_1, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL);
	PIN_Configure (ADC_PIN[1].Portnum, ADC_PIN[1].Pinnum, PIN_FUNC_1, PIN_PINMODE_TRISTATE, PIN_PINMODE_NORMAL);
	LPC_ADC->ADCR    =  ( 1 <<  0) | ( 1 <<  1) | ( 4 <<  8) | ( 1 << 16) | ( 1 << 21);    
	LPC_ADC->ADINTEN =  ( 1 <<  8);
	NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_Uninitialize (void) {
	NVIC_DisableIRQ (ADC_IRQn);
	LPC_ADC->ADINTEN &= ~( 1 <<  8);
	PIN_Configure (ADC_PIN[0].Portnum, ADC_PIN[0].Pinnum, 0, 0, 0);
	PIN_Configure (ADC_PIN[1].Portnum, ADC_PIN[1].Pinnum, 0, 0, 0); 
	LPC_SC->PCONP &= ~(1 << 12);
}

void ADC_StartBurstConversion (void) {
	LPC_ADC->ADCR |=  ( 1 << 16);
}

void ADC_StopBurstConversion (void) {
	LPC_ADC->ADCR &= ~( 0 << 16);
}

void motorLeft(bool shutdown, int throttle) {
	int8_t value[2];
	value[0] = !shutdown << 4 | throttle >> 6;
	value[1] = throttle << 2;

	Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	Driver_SPI1.Send(value, sizeof(value));
	Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}

void motorRight(bool shutdown, int throttle) {
	int8_t value[2];
	value[0] = 0x08 << 4 | !shutdown << 4 | throttle >> 6;
	value[1] = (throttle & 0xFF) << 2;

	Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
	Driver_SPI1.Send(value, sizeof(value));
	Driver_SPI1.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
}

void heartBeatThread(void *arg) {
	while (1) {
		GPIO_PinWrite(2, 7, 0);
		osDelay(10);
		GPIO_PinWrite(2, 7, 1);
		osDelay(1000);
	}
}

void filterEngine(void *arg) {
	while (1) {
		osSemaphoreAcquire(semaphoreAdcId, osWaitForever);
		axisX = ((channel1 + ((LPFTap - 1) * axisX))/LPFTap);
		axisY = ((channel2 + ((LPFTap - 1) * axisY))/LPFTap);
	}
}

void computeEngine(void *arg) {
	while (1) {
		osSemaphoreAcquire(semaphoreAdcId, osWaitForever);
		motorLeft(false, (axisX >> 2));
		motorRight(false, (axisY >> 2));
		osDelay(300);
	}
}

void Debug(void *arg) {
	while (1) {
		if (AD_done) {
			itmPrint((char *) "\naxisX: ");
			itmPrintlnInt((axisX * 3300)/4096);
			// itmPrint((char *) "axisY: ");
			// itmPrintlnInt((axisY * 3300)/4096);
		}
		osDelay(200);
	}
}

int main(void) {

	SystemCoreClockUpdate ();
	SysTick_Config(SystemCoreClock/1000);

	osKernelInitialize();
	uart0Initialize();

	Driver_USART0.Send("\nSystem Initialize\n", 19);

	ledInitialize();
	spi1Initialize();
	ADC_Initialize();

	semaphoreAdcAttr.name = "ADC Semaphore";
	semaphoreAdcId = osSemaphoreNew(1, 0, &semaphoreAdcAttr);
 
	osThreadAttr_t osThreadHeartBeatAttr;
	osThreadHeartBeatAttr.name = "HeartBeat";
	osThreadHeartBeatAttr.priority = osPriorityNormal;
	osThreadHeartBeatAttr.stack_size = 128;
	osThreadHeartBeatAttr.cb_mem = 0;
	osThreadHeartBeatAttr.cb_size = 0;
	osThreadHeartBeatAttr.stack_mem = 0;

	osThreadAttr_t osThreadComputeEngineAttr;
	osThreadComputeEngineAttr.name = "Compute Engine";
	osThreadComputeEngineAttr.priority = osPriorityNormal;
	osThreadComputeEngineAttr.stack_size = 1000;
	osThreadComputeEngineAttr.cb_mem = 0;
	osThreadComputeEngineAttr.cb_size = 0;
	osThreadComputeEngineAttr.stack_mem = 0;

	osThreadAttr_t osThreadDebugAttr;
	osThreadDebugAttr.name = "Debug";
	osThreadDebugAttr.priority = osPriorityNormal;
	osThreadDebugAttr.stack_size = 1000;
	osThreadDebugAttr.cb_mem = 0;
	osThreadDebugAttr.cb_size = 0;
	osThreadDebugAttr.stack_mem = 0;

	osThreadNew(heartBeatThread, NULL, &osThreadHeartBeatAttr);
	osThreadNew(filterEngine, NULL, &osThreadComputeEngineAttr);
	osThreadNew(computeEngine, NULL, &osThreadComputeEngineAttr);
	osThreadNew(Debug, NULL, &osThreadDebugAttr);

	osKernelStart();
}
