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
#define LEFT_CONTROL_1 0 // Port B Pin 0 -> TPM1_CH0
#define RIGHT_CONTROL_1 1 // Port B Pin 1 -> TPM1_CH1
#define LEFT_CONTROL_2 2 // Port B Pin 2 -> TPM2_CH0
#define RIGHT_CONTROL_2 3 // Port B Pin 3 -> TPM2_CH1
#define MUSIC_PIN 0 // Port D Pin 0 -> TPM0_CH0
#define MSG_COUNT 1

// Port C Pins
#define GREEN_LED_1 7
#define GREEN_LED_2 0
#define GREEN_LED_3 3
#define GREEN_LED_4 4
#define GREEN_LED_5 5
#define GREEN_LED_6 6
#define GREEN_LED_7 10
#define GREEN_LED_8 11
#define GREEN_LED_9 12
#define GREEN_LED_10 13
#define RED_LED 16

osThreadId_t forwardId, leftId, rightId, reverseId, stopId, controlId, greenLedId, redLedId, playMusicId;
osMessageQueueId_t forwardMsg, leftMsg, rightMsg, reverseMsg, stopMsg, greenLedMsg, redLedMsg, playMusicMsg;

typedef struct {
	uint8_t cmd;
} dataPacket;

enum commands {
	forward = 1,
	left = 2,
	right = 3,
	reverse = 4,
	stop = 5,
	playMusic = 6,
	stopMusic = 7
};

dataPacket receivedData;
uint8_t robotMovingStatus = 0; // 0 means stopped and 1 means moving
uint8_t playFinalMusic = 0; // 0 means play normal music and 1 means play final music
uint8_t greenLedCounter = 0;
int musicalNotesFrequencies[7][2] = {{'C', 262}, {'D', 294}, {'E', 330}, {'F', 349}, {'G', 392}, {'A', 440}, {'B', 494}};

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

void initLED(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;

	// Configure MUX settings for LED pins
	PORTC->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1); // Setting Alernative 1

	PORTC->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1); // Setting Alernative 1
	
	PORTC->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1); // Setting Alernative 1

	PORTC->PCR[GREEN_LED_4] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1); // Setting Alernative 1
	
	PORTC->PCR[GREEN_LED_5] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1); // Setting Alernative 1
	
	PORTC->PCR[GREEN_LED_6] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1); // Setting Alernative 1
	
	PORTC->PCR[GREEN_LED_7] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1); // Setting Alernative 1
	
	PORTC->PCR[GREEN_LED_8] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[GREEN_LED_8] |= PORT_PCR_MUX(1); // Setting Alernative 1
	
	PORTC->PCR[GREEN_LED_9] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[GREEN_LED_9] |= PORT_PCR_MUX(1); // Setting Alernative 1
	
	PORTC->PCR[GREEN_LED_10] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[GREEN_LED_10] |= PORT_PCR_MUX(1); // Setting Alernative 1
	
	PORTC->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[RED_LED] |= PORT_PCR_MUX(1); // Setting Alernative 1
	
	// Set Data Direction Registers for Port C (Set them as outputs)
	PTC->PDDR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8) | MASK(GREEN_LED_9) | MASK(GREEN_LED_10) | MASK(RED_LED));
}

void initMusicPin(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	// Configure MUX settings for music pin
	PORTC->PCR[MUSIC_PIN] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTC->PCR[MUSIC_PIN] |= PORT_PCR_MUX(4); // Setting Alernative 4 -> TPM0_CH0
	
	// Enable Clock to TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	// MCGFLCLK or MCGPLLCLK/2 - Select internal clock
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); 
	
	// Set MOD values for TPM1 and TPM2
	TPM0->MOD = 375000;
	
	/*
	Edge aligned PWM:
	Update SnC register to CMOD = 01 and PS = 111 (128)
	*/
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Enable PWM on TPM1 Channel 0 -> PTD0
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | (TPM_CnSC_MSB(1)));
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

void offAllGreenLeds(void) {
	// Turn off all Green LEDs
	PTC->PCOR = (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8) | MASK(GREEN_LED_9) | MASK(GREEN_LED_10));
}

void onAllGreenLeds(void) {
  // Turn on all Green LEDs
	PTC->PSOR = (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3) | MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8) | MASK(GREEN_LED_9) | MASK(GREEN_LED_10));
}

void incrementGreenLedCounter(void) {
	greenLedCounter++;
	if (greenLedCounter > 10) {
		greenLedCounter = 1;
	}
}

void greenLedControl() {
	if (!robotMovingStatus) {
		// Stationery
		onAllGreenLeds();
	} else {
		//Moving
		switch(greenLedCounter) {
			case 1:
				offAllGreenLeds();
				PTC->PSOR = MASK(GREEN_LED_1);
				incrementGreenLedCounter();	
				osDelay(1000);
				break;
			case 2:
				offAllGreenLeds();
				PTC->PSOR = MASK(GREEN_LED_2);
				incrementGreenLedCounter();	
				osDelay(1000);
				break;
			case 3:
				offAllGreenLeds();
				PTC->PSOR = MASK(GREEN_LED_3);
				incrementGreenLedCounter();	
				osDelay(1000);
				break;
			case 4:
				offAllGreenLeds();
				PTC->PSOR = MASK(GREEN_LED_4);
				incrementGreenLedCounter();	
				osDelay(1000);
				break;
			case 5:
				offAllGreenLeds();
				PTC->PSOR = MASK(GREEN_LED_5);
				incrementGreenLedCounter();	
				osDelay(1000);
				break;
			case 6:
				offAllGreenLeds();
				PTC->PSOR = MASK(GREEN_LED_6);
				incrementGreenLedCounter();	
				osDelay(1000);
				break;
			case 7:
				offAllGreenLeds();
				PTC->PSOR = MASK(GREEN_LED_7);
				incrementGreenLedCounter();	
				osDelay(1000);
				break;
			case 8:
				offAllGreenLeds();
				PTC->PSOR = MASK(GREEN_LED_8);
				incrementGreenLedCounter();	
				osDelay(1000);
				break;
			case 9:
				offAllGreenLeds();
				PTC->PSOR = MASK(GREEN_LED_9);
				incrementGreenLedCounter();	
				osDelay(1000);
				break;
			case 10:
				offAllGreenLeds();
				PTC->PSOR = MASK(GREEN_LED_10);
				incrementGreenLedCounter();	
				osDelay(1000);
				break;
		}
	}
}

void redLedControl(void) {
	if (!robotMovingStatus) {
		// Stationery
		PTC->PSOR = MASK(RED_LED);
		osDelay(250);
		PTC->PCOR = MASK(RED_LED);
		osDelay(250);
	} else {
		// Moving
		PTC->PSOR = MASK(RED_LED);
		osDelay(500);
		PTC->PCOR = MASK(RED_LED);
		osDelay(500);
	}
}

void motorControl (int cmd) {
	int leftFrequency;
	int rightFrequency;
	switch(cmd) {
		case forward:
			leftFrequency = 5;
			rightFrequency = 5;
			TPM1_C0V = (375000 / leftFrequency) / 2;
			TPM1_C1V = (375000 / rightFrequency) / 2;
			TPM2_C0V = 0;
			TPM2_C1V = 0;
			break;
		case left:
			leftFrequency = 20;
			rightFrequency = 5;
			TPM1_C0V = (375000 / leftFrequency) / 2;
			TPM1_C1V = (375000 / rightFrequency) / 2;
			TPM2_C0V = 0;
			TPM2_C1V = 0;
			break;
		case right:		
			leftFrequency = 5;
			rightFrequency = 20;
			TPM1_C0V = (375000 / leftFrequency) / 2;
			TPM1_C1V = (375000 / rightFrequency) / 2;
			TPM2_C0V = 0;
			TPM2_C1V = 0;
			break;
		case reverse:
			leftFrequency = 5;
			rightFrequency = 5;
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

void musicControl(void) {
	if(playFinalMusic) { // Final Music
		// Note C
		TPM0->MOD = 375000 / musicalNotesFrequencies[0][1];
		TPM0_C0V = (375000 / musicalNotesFrequencies[0][1]) / 2;
	} else { // Normal Music
		// Note B
		TPM0->MOD = 375000 / musicalNotesFrequencies[6][1];
		TPM0_C0V = (375000 / musicalNotesFrequencies[6][1]) / 2;
	}
}

/*----------------------------------------------------------------------------
 * Main Control Thread
 *---------------------------------------------------------------------------*/
void control_thread(void *argument) {
  for (;;) {
		if(receivedData.cmd == forward) {
			robotMovingStatus = 1;
			osMessageQueuePut(forwardMsg, &receivedData, NULL, 0);
		} 
		if (receivedData.cmd == left) {
			robotMovingStatus = 1;
			osMessageQueuePut(leftMsg, &receivedData, NULL, 0);
		} 
		if (receivedData.cmd == right) {
			robotMovingStatus = 1;
			osMessageQueuePut(rightMsg, &receivedData, NULL, 0);
		} 
		if (receivedData.cmd == reverse) {
			robotMovingStatus = 1;
			osMessageQueuePut(reverseMsg, &receivedData, NULL, 0);
		} 
		if (receivedData.cmd == stop) {
			robotMovingStatus = 0;
			osMessageQueuePut(stopMsg, &receivedData, NULL, 0);
		}
		if (receivedData.cmd == playMusic) {
			playFinalMusic = 1;
		} else if (receivedData.cmd == stopMusic){
			playFinalMusic = 0;
		} 
		
		osMessageQueuePut(playMusicMsg, NULL, NULL, 0);		
		osMessageQueuePut(greenLedMsg, NULL, NULL, 0);
		osMessageQueuePut(redLedMsg, NULL, NULL, 0);
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

void green_led_thread(void *argument) {
  for (;;) {
		greenLedControl();
  }
}

void red_led_thread(void *argument) {
  for (;;) {
		redLedControl();
  }
}

void play_music_thread(void *argument) {
  for (;;) {
		musicControl();
  }
}

int main(void) {
	// System Initialization
	SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
	initGPIO();
	initLED();
	initMusicPin();
	
	osKernelInitialize();  // Initialize CMSIS-RTOS
	forwardId = osThreadNew(forward_thread, NULL, NULL);
	leftId = osThreadNew(left_thread, NULL, NULL);
	rightId = osThreadNew(right_thread, NULL, NULL);
	reverseId = osThreadNew(reverse_thread, NULL, NULL);
	stopId = osThreadNew(stop_thread, NULL, NULL);
	controlId = osThreadNew(control_thread, NULL, NULL);
	greenLedId = osThreadNew(green_led_thread, NULL, NULL);
	redLedId = osThreadNew(red_led_thread, NULL, NULL);
	playMusicId = osThreadNew(play_music_thread, NULL, NULL);
	
	forwardMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	leftMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	rightMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	reverseMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	stopMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	greenLedMsg = osMessageQueueNew(MSG_COUNT, sizeof(uint8_t), NULL);
	redLedMsg = osMessageQueueNew(MSG_COUNT, sizeof(uint8_t), NULL);
	playMusicMsg = osMessageQueueNew(MSG_COUNT, sizeof(uint8_t), NULL);
	
	osKernelStart();  // Start thread execution
	for (;;) {}
}
