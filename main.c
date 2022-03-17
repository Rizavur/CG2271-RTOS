/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"

#define MASK(x)(1<<(x))
#define BAUD_RATE 9600
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128
#define LEFT_CONTROL_1 0 // Port B Pin 0
#define RIGHT_CONTROL_1 1 // Port B Pin 1
#define LEFT_CONTROL_2 2 // Port B Pin 2
#define RIGHT_CONTROL_2 3 // Port B Pin 3
#define MSG_COUNT 1

osThreadId_t forwardId, leftId, rightId, reverseId, stopId, controlId;
osMessageQueueId_t forwardMsg, leftMsg, rightMsg, reverseMsg, stopMsg;

typedef struct {
	uint8_t cmd;
} dataPacket;

enum commands {
	forward = 1,
	left = 2,
	right = 3,
	reverse = 4,
	stop = 5
};

dataPacket receivedData;

void initGPIO(void) {
	// Enable Clock to PORTB 
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[LEFT_CONTROL_1] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTB->PCR[LEFT_CONTROL_1] |= PORT_PCR_MUX(3); // Setting Alernative 3 Timer (PWM)

	PORTB->PCR[RIGHT_CONTROL_1] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTB->PCR[RIGHT_CONTROL_1] |= PORT_PCR_MUX(3); // Setting Alernative 3 Timer (PWM)
	
	PORTB->PCR[LEFT_CONTROL_2] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTB->PCR[LEFT_CONTROL_2] |= PORT_PCR_MUX(3); // Setting Alernative 3 Timer (PWM)

	PORTB->PCR[RIGHT_CONTROL_2] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTB->PCR[RIGHT_CONTROL_2] |= PORT_PCR_MUX(3); // Setting Alernative 3 Timer (PWM)

	// Enable Clock to TPM1
	SIM->SCGC6 |= (SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM2_MASK);
	
	// MCGFLCLK or MCGPLLCLK/2 - Select internal clock
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); 
	
	// Set MOD values for TPM1 and TPM2
	TPM1->MOD = 375000;
	TPM2->MOD = 375000;
	
	/*
	Edge aligned PWM:
	Update SnC register to CMOD = 01 and PS = 111 (128)
	*/
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Enable PWM on TPM1 Channel 0 -> PTB0
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | (TPM_CnSC_MSB(1)));
	
	// Enable PWM on TPM1 Channel 1 -> PTB1
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1) | (TPM_CnSC_MSB(1)));
	
	// Enable PWM on TPM2 Channel 0 -> PTB2
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | (TPM_CnSC_MSB(1)));
	
	// Enable PWM on TPM2 Channel 1 -> PTB3
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | (TPM_CnSC_MSB(1)));
}

void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;

	// Enable clock to UART and Port E
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// Connect UART to pins for PTE 23 (Rx)
	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);
	
	// Ensure that Rx are disabled before configuration
	UART2->C2 &= ~UART_C2_RE_MASK;
	
	// Set baud rate to 9600 baud
	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	// No parity
	UART2->C1 = 0;
	// 8 Bits
	UART2->S2 = 0;
	// 2 Stop bits
	UART2->C3 = 0;
	
	// Enable Rx
	UART2->C2 |= UART_C2_RE_MASK;
	
	// For interrupts with UART
	NVIC_SetPriority(UART2_IRQn, UART2_INT_PRIO);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_RIE_MASK;
}

void UART2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(UART2_IRQn);
		
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		dataPacket received;
		received.cmd = UART2->D;
		receivedData = received;
	}
	
	if (UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
		// Handle the error here
	}
}

void motorControl (int cmd) {
	int leftFrequency;
	int rightFrequency;
	switch(cmd) {
		case forward:
			leftFrequency = 100;
			rightFrequency = 100;
			TPM1_C0V = (375000 / leftFrequency) / 2;
			TPM1_C1V = (375000 / rightFrequency) / 2;
			TPM2_C0V = 0;
			TPM2_C1V = 0;
			break;
		case left: 
			leftFrequency = 50;
			rightFrequency = 100;
			TPM1_C0V = (375000 / leftFrequency) / 2;
			TPM1_C1V = (375000 / rightFrequency) / 2;
			TPM2_C0V = 0;
			TPM2_C1V = 0;
			break;
		case right: 
			leftFrequency = 100;
			rightFrequency = 50;
			TPM1_C0V = (375000 / leftFrequency) / 2;
			TPM1_C1V = (375000 / rightFrequency) / 2;
			TPM2_C0V = 0;
			TPM2_C1V = 0;
			break;
		case reverse:
			leftFrequency = 100;
			rightFrequency = 50;
			TPM1_C0V = 0;
			TPM1_C1V = 0;
			TPM2_C0V = (375000 / leftFrequency) / 2;
			TPM2_C1V = (375000 / rightFrequency) / 2;
			break;
		case stop:
			TPM1_C0V = 0;
			TPM1_C1V = 0;
			TPM2_C0V = 0;
			TPM2_C1V = 0;
			break;
	}
}

/*----------------------------------------------------------------------------
 * Main Control Thread
 *---------------------------------------------------------------------------*/
void control_thread(void *argument) {
  for (;;) {
		if(receivedData.cmd == forward) {
			osMessageQueuePut(forwardMsg, &receivedData, NULL, 0);
		} else if (receivedData.cmd == left) {
			osMessageQueuePut(leftMsg, &receivedData, NULL, 0);
		} else if (receivedData.cmd == right) {
			osMessageQueuePut(rightMsg, &receivedData, NULL, 0);
		} else if (receivedData.cmd == reverse) {
			osMessageQueuePut(reverseMsg, &receivedData, NULL, 0);
		} else if (receivedData.cmd == stop) {
			osMessageQueuePut(stopMsg, &receivedData, NULL, 0);
		}
  }
}

/*----------------------------------------------------------------------------
 * Move Robot Forward
 *---------------------------------------------------------------------------*/
void forward_thread(void *argument) {
  dataPacket rxData;
  for (;;) {
		osMessageQueueGet(forwardMsg, &rxData, NULL, osWaitForever);
		motorControl(forward);
  }
}

/*----------------------------------------------------------------------------
 * Move Robot Left
 *---------------------------------------------------------------------------*/
void left_thread(void *argument) {
  dataPacket rxData;
  for (;;) {
		osMessageQueueGet(leftMsg, &rxData, NULL, osWaitForever);
		motorControl(left);
  }
}

/*----------------------------------------------------------------------------
 * Move Robot Right
 *---------------------------------------------------------------------------*/
void right_thread(void *argument) {
  dataPacket rxData;
  for (;;) {
		osMessageQueueGet(rightMsg, &rxData, NULL, osWaitForever);
		motorControl(right);
  }
}

/*----------------------------------------------------------------------------
 * Move Robot Backwards
 *---------------------------------------------------------------------------*/
void reverse_thread(void *argument) {
  dataPacket rxData;
  for (;;) {
		osMessageQueueGet(reverseMsg, &rxData, NULL, osWaitForever);
		motorControl(reverse);
  }
}

/*----------------------------------------------------------------------------
 * Stop Robot
 *---------------------------------------------------------------------------*/
void stop_thread(void *argument) {
  dataPacket rxData;
  for (;;) {
		osMessageQueueGet(stopMsg, &rxData, NULL, osWaitForever);
		motorControl(stop);
  }
}

int main(void) {
  // System Initialization
  SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
	initGPIO();
  // ...

  osKernelInitialize();  // Initialize CMSIS-RTOS
  forwardId = osThreadNew(forward_thread, NULL, NULL);
  leftId = osThreadNew(left_thread, NULL, NULL);
  rightId = osThreadNew(right_thread, NULL, NULL);
  reverseId = osThreadNew(reverse_thread, NULL, NULL);
	stopId = osThreadNew(stop_thread, NULL, NULL);
  controlId = osThreadNew(control_thread, NULL, NULL);
	
	forwardMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	leftMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	rightMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	reverseMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	stopMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	
  osKernelStart();  // Start thread execution
  for (;;) {
  }
}
