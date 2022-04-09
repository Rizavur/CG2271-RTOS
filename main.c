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
#define LEFT_CONTROL_1 0 // Port D Pin 0 -> TPM0_CH0
#define LEFT_CONTROL_2 1 // Port D Pin 1 -> TPM0_CH1
#define RIGHT_CONTROL_1 2 // Port D Pin 2 -> TPM0_CH2
#define RIGHT_CONTROL_2 3 // Port D Pin 3 -> TPM0_CH3
#define MUSIC_PIN 0 // Port B Pin 0 -> TPM1_CH0
#define MSG_COUNT 1
#define ULTRASONIC_ECHO 3 // Port B Pin 3 -> TPM2_CH1
#define ULTRASONIC_TRIGGER 2 // Port B Pin 2 -> TPM2_CH0

// Port C Pins
#define GREEN_LED_1 1
#define GREEN_LED_2 2
#define GREEN_LED_3 12
#define GREEN_LED_4 4
#define GREEN_LED_5 5
#define GREEN_LED_6 6
#define GREEN_LED_7 10
#define GREEN_LED_8 11
#define GREEN_LED_9 12
#define GREEN_LED_10 13
#define RED_LED 16

#define NOTE_G4  392
#define NOTE_AS4 466
#define NOTE_C5  523
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_C6  1047
#define NOTE_F6  1397

osThreadId_t motorId, controlId, greenLedId, redLedId, playMusicId, autonomousModeId, ultrasonicId, manualStopId;
osMessageQueueId_t motorMsg;
osSemaphoreId_t autonomousModeSem, autonomousStopSem, manualModeSem, manualStopSem;
osThreadAttr_t highPriority = {.priority = osPriorityHigh};

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
	stopMusic = 7,
	autonomousMode = 8,
	smallLeft = 9,
	smallRight = 10,
	smallForward = 11
};

dataPacket receivedData;
uint8_t robotMovingStatus = 0; // 0 means stopped and 1 means moving
uint8_t playFinalMusic = 0; // 0 means play normal music and 1 means play final music
volatile int ultrasonicFlag = 0, ultrasonicStart = 1, ultrasonicValue = 0;

void initMotorPins(void) {
	// Enable Clock to PORTD
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

	// Configure MUX settings to make all 3 pins GPIO
	PORTD->PCR[LEFT_CONTROL_1] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTD->PCR[LEFT_CONTROL_1] |= PORT_PCR_MUX(4); // Setting Alernative 4 Timer (PWM)

	PORTD->PCR[RIGHT_CONTROL_1] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTD->PCR[RIGHT_CONTROL_1] |= PORT_PCR_MUX(4); // Setting Alernative 4 Timer (PWM)
	
	PORTD->PCR[LEFT_CONTROL_2] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTD->PCR[LEFT_CONTROL_2] |= PORT_PCR_MUX(4); // Setting Alernative 4 Timer (PWM)

	PORTD->PCR[RIGHT_CONTROL_2] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTD->PCR[RIGHT_CONTROL_2] |= PORT_PCR_MUX(4); // Setting Alernative 4 Timer (PWM)

	// Enable Clock to TPM0
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	// MCGFLCLK or MCGPLLCLK/2 - Select internal clock
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); 
	
	// Set MOD value for TPM0
	TPM0->MOD = 375000;
	
	/*
	Edge aligned PWM:
	Update SnC register to CMOD = 01 and PS = 111 (128)
	*/
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Enable PWM on TPM0 Channel 0 -> PTD0
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | (TPM_CnSC_MSB(1)));
	
	// Enable PWM on TPM0 Channel 1 -> PTD1
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | (TPM_CnSC_MSB(1)));
	
	// Enable PWM on TPM0 Channel 2 -> PTD2
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | (TPM_CnSC_MSB(1)));
	
	// Enable PWM on TPM0 Channel 3 -> PTD3
	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | (TPM_CnSC_MSB(1)));
}

void initLEDPins(void) {
	// Enable clock to Port C and Port A
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

	// Configure MUX settings for LED pins
	PORTA->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTA->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1); // Setting Alernative 1

	PORTA->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTA->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1); // Setting Alernative 1
	
	PORTA->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTA->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1); // Setting Alernative 1

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
	PTA->PDDR |= (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3));
	PTC->PDDR |= (MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8) | MASK(GREEN_LED_9) | MASK(GREEN_LED_10) | MASK(RED_LED));
}

void initMusicPin(void) {
	// Enable clock to Port B
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// Configure MUX settings for music pin
	PORTB->PCR[MUSIC_PIN] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTB->PCR[MUSIC_PIN] |= PORT_PCR_MUX(3); // Setting Alernative 3 -> TPM1_CH0
	
	// Enable Clock to TPM1
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	// MCGFLCLK or MCGPLLCLK/2 - Select internal clock
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); 
	
	// Set MOD values for TPM1
	TPM1->MOD = 375000;
	
	/*
	Edge aligned PWM:
	Update SnC register to CMOD = 01 and PS = 111 (128)
	*/
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	// Enable PWM on TPM1 Channel 0 -> PTD0
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | (TPM_CnSC_MSB(1)));
}

void initUltrasonic() {
	// Enable clock to Port B
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// Enable clock to TPM2
	SIM_SCGC6 |= SIM_SCGC6_TPM2_MASK;
 
 	// MCGFLCLK or MCGPLLCLK/2 - Select internal clock
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
 
	PORTB->PCR[ULTRASONIC_ECHO] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTB->PCR[ULTRASONIC_ECHO] |= PORT_PCR_MUX(3); // Setting Alernative 1 -> TPM2_C0
	PORTB->PCR[ULTRASONIC_TRIGGER] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTB->PCR[ULTRASONIC_TRIGGER] |= PORT_PCR_MUX(3); // Setting Alernative 1 -> TPM2_C1
	
	/*
	Edge aligned PWM:
	Update SnC register to CMOD = 01 and PS = 101 (32)
	*/
	TPM2_SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK | TPM_SC_CPWMS_MASK);
	TPM2_SC |= TPM_SC_PS(5);
	
	// Enable interrupt for TPM2_CH1 (Echo Pin)
	TPM2_C1SC |= TPM_CnSC_CHIE_MASK;
	
	// Set MOD value for TPM2 
	TPM2_MOD = 10000;
	
	// After prescaler, 48Mhz --> 1.5Mhz. Need pulse every 10 micro sec --> frequency = 100Khz.
	// CnV value needed = 1.5Mhz/100Khz = 15. (For Trigger pin)
	TPM2_C0V = 15;
		
	NVIC_SetPriority(TPM2_IRQn, 0);
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
		if (received.cmd == autonomousMode) {
			osSemaphoreRelease(autonomousModeSem);
		} else if (received.cmd == stop) {
			osSemaphoreRelease(manualStopSem);
		} else {
			osSemaphoreRelease(manualModeSem);
		}
	}
	
	if (UART2->S1 & (UART_S1_OR_MASK | UART_S1_NF_MASK | UART_S1_FE_MASK | UART_S1_PF_MASK)) {
		// Handle the error here
	}
}

void TPM2_IRQHandler(void) {
	NVIC_ClearPendingIRQ(TPM2_IRQn);
	
	if (ultrasonicStart) {
		// Reset TPM2 count
		TPM2_CNT = 0;

		ultrasonicStart = 0;

		// Capture on falling edges for TPM2_C1 (Echo Pin)
		TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
		TPM2_C1SC |= TPM_CnSC_ELSB_MASK;
	} else {
		ultrasonicValue = TPM2_C1V;
		// If ultrasonic required and object is detected within range
		if (ultrasonicFlag && ultrasonicValue <= 4200 && ultrasonicValue >= 600) {
				osSemaphoreRelease(autonomousStopSem);
				ultrasonicFlag = 0;
		}
		ultrasonicStart = 1;
		NVIC_DisableIRQ(TPM2_IRQn);
	}
}

void offAllGreenLeds(void) {
	// Turn off all Green LEDs
	PTA->PCOR = (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3));
	PTC->PCOR = (MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8) | MASK(GREEN_LED_9) | MASK(GREEN_LED_10));
}

void onAllGreenLeds(void) {
  // Turn on all Green LEDs
	PTA->PSOR = (MASK(GREEN_LED_1) | MASK(GREEN_LED_2) | MASK(GREEN_LED_3));
	PTC->PSOR = (MASK(GREEN_LED_4) | MASK(GREEN_LED_5) | MASK(GREEN_LED_6) | MASK(GREEN_LED_7) | MASK(GREEN_LED_8) | MASK(GREEN_LED_9) | MASK(GREEN_LED_10));
}

void greenLedControl() {
  if (!robotMovingStatus) {
    // Stationery
    onAllGreenLeds();
  } else {
    //Moving
		offAllGreenLeds();
		PTA->PSOR = MASK(GREEN_LED_1);
		osDelay(20);
		
		offAllGreenLeds();
		PTA->PSOR = MASK(GREEN_LED_2);
		osDelay(20);
		
		offAllGreenLeds();
		PTA->PSOR = MASK(GREEN_LED_3);
		osDelay(20);
		
		offAllGreenLeds();
		PTC->PSOR = MASK(GREEN_LED_4);
		osDelay(20);
		
		offAllGreenLeds();
		PTC->PSOR = MASK(GREEN_LED_5);
		osDelay(20);
		
		offAllGreenLeds();
		PTC->PSOR = MASK(GREEN_LED_6);
		osDelay(20);
		
		offAllGreenLeds();
		PTC->PSOR = MASK(GREEN_LED_7);  
		osDelay(20);
		
		offAllGreenLeds();
		PTC->PSOR = MASK(GREEN_LED_8); 
		osDelay(20);
		
		offAllGreenLeds();
		PTC->PSOR = MASK(GREEN_LED_9); 
		osDelay(20);
		
		offAllGreenLeds();
		PTC->PSOR = MASK(GREEN_LED_10);
		osDelay(20);

		offAllGreenLeds();
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

void autonomousMotorControl(int cmd) {
	int leftFrequency;
	int rightFrequency;
	switch(cmd) {
		case forward:
			leftFrequency = 10;
			rightFrequency = 10;
			TPM0_C0V = (375000 / leftFrequency) / 3;
			TPM0_C2V = (375000 / rightFrequency) / 3;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
		case left:
			leftFrequency = 5;
			rightFrequency = 5;
			TPM0_C0V = 0;
			TPM0_C2V = (375000 / rightFrequency) / 5;
			TPM0_C1V = (375000 / leftFrequency) / 5;
			TPM0_C3V = 0;
		case right:
			leftFrequency = 5;
			rightFrequency = 5;
			TPM0_C0V = (375000 / leftFrequency) / 5;
			TPM0_C2V = 0;
			TPM0_C1V = 0;
			TPM0_C3V = (375000 / rightFrequency) / 5;
			break;
		case reverse:
			leftFrequency = 8;
			rightFrequency = 8;
			TPM0_C0V = 0;
			TPM0_C2V = 0;
			TPM0_C1V = (375000 / leftFrequency) / 3;
			TPM0_C3V = (375000 / rightFrequency) / 3;
			break;
		case stop:
			TPM0_C0V = 0;
			TPM0_C2V = 0;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
	}
}

void motorControl (int cmd) {
	int leftFrequency;
	int rightFrequency;
	switch(cmd) {
		case forward:
			leftFrequency = 5;
			rightFrequency = 5;
			TPM0_C0V = (375000 / leftFrequency) / 3;
			TPM0_C2V = (375000 / rightFrequency) / 3;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
		case left:
			leftFrequency = 20;
			rightFrequency = 5;
			TPM0_C0V = (375000 / leftFrequency) / 3;
			TPM0_C2V = (375000 / rightFrequency) / 3;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
		case right:		
			leftFrequency = 5;
			rightFrequency = 20;
			TPM0_C0V = (375000 / leftFrequency) / 3;
			TPM0_C2V = (375000 / rightFrequency) / 3;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
		case reverse:
			leftFrequency = 5;
			rightFrequency = 5;
			TPM0_C0V = 0;
			TPM0_C2V = 0;
			TPM0_C1V = (375000 / leftFrequency) / 3;
			TPM0_C3V = (375000 / rightFrequency) / 3;
			break;
		case smallLeft:
			leftFrequency = 0;
			rightFrequency = 15;
			TPM0_C0V = (375000 / leftFrequency) / 3;
			TPM0_C2V = (375000 / rightFrequency) / 3;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
		case smallRight:		
			leftFrequency = 15;
			rightFrequency = 0;
			TPM0_C0V = (375000 / leftFrequency) / 3;
			TPM0_C2V = (375000 / rightFrequency) / 3;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
		case smallForward:		
			leftFrequency = 15;
			rightFrequency = 15;
			TPM0_C0V = (375000 / leftFrequency) / 3;
			TPM0_C2V = (375000 / rightFrequency) / 3;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
		case stop:
			TPM0_C0V = 0;
			TPM0_C2V = 0;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;

	}
}

void finalMusic() {
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_AS4;
	TPM1_C0V = (375000 / NOTE_AS4) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_AS4;
	TPM1_C0V = (375000 / NOTE_AS4) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_AS4;
	TPM1_C0V = (375000 / NOTE_AS4) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_F5;
	TPM1_C0V = (375000 / NOTE_F5) / 4;
	osDelay(40);
		
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_C6;
	TPM1_C0V = (375000 / NOTE_C6) / 4;
	osDelay(40);
		
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_AS5;
	TPM1_C0V = (375000 / NOTE_AS5) / 4;
	osDelay(160);
		
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_A5;
	TPM1_C0V = (375000 / NOTE_A5) / 4;
	osDelay(160);
		
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_G5;
	TPM1_C0V = (375000 / NOTE_G5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_F6;
	TPM1_C0V = (375000 / NOTE_F6) / 4;
	osDelay(40);
		
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_C6;
	TPM1_C0V = (375000 / NOTE_C6) / 4;
	osDelay(80);
		
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_AS5;
	TPM1_C0V = (375000 / NOTE_AS5) / 4;
	osDelay(160);
		
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_A5;
	TPM1_C0V = (375000 / NOTE_A5) / 4;
	osDelay(160);
		
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_G5;
	TPM1_C0V = (375000 / NOTE_G5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_F6;
	TPM1_C0V = (375000 / NOTE_F6) / 4;
	osDelay(40);
		
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_C6;
	TPM1_C0V = (375000 / NOTE_C6) / 4;
	osDelay(80);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_AS5;
	TPM1_C0V = (375000 / NOTE_AS5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_A5;
	TPM1_C0V = (375000 / NOTE_A5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_AS5;
	TPM1_C0V = (375000 / NOTE_AS5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_G5;
	TPM1_C0V = (375000 / NOTE_G5) / 4;
	osDelay(40);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_C5;
	TPM1_C0V = (375000 / NOTE_C5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_C5;
	TPM1_C0V = (375000 / NOTE_C5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_C5;
	TPM1_C0V = (375000 / NOTE_C5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_C5;
	TPM1_C0V = (375000 / NOTE_C5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_F5;
	TPM1_C0V = (375000 / NOTE_F5) / 4;
	osDelay(40);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_C6;
	TPM1_C0V = (375000 / NOTE_C6) / 4;
	osDelay(40);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_AS5;
	TPM1_C0V = (375000 / NOTE_AS5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_A5;
	TPM1_C0V = (375000 / NOTE_A5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_G5;
	TPM1_C0V = (375000 / NOTE_G5) / 4;
	osDelay(160);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_F6;
	TPM1_C0V = (375000 / NOTE_F6) / 4;
	osDelay(40);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_C6;
	TPM1_C0V = (375000 / NOTE_C6) / 4;
	osDelay(80);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
}

void normalMusic() {
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 375000 / NOTE_E5;
  TPM1_C0V = (375000 / NOTE_E5) / 4;
  osDelay(160);

	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
  TPM1->MOD = 375000 / NOTE_E5;
  TPM1_C0V = (375000 / NOTE_E5) / 4;
  osDelay(160);

	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(160);

	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
  TPM1->MOD = 375000 / NOTE_E5;
  TPM1_C0V = (375000 / NOTE_E5) / 4;
  osDelay(160);

	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(160);

	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
  TPM1->MOD = 375000 / NOTE_C5;
  TPM1_C0V = (375000 / NOTE_C5) / 4;
  osDelay(160);

	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
  TPM1->MOD = 375000 / NOTE_E5;
  TPM1_C0V = (375000 / NOTE_E5) / 4;
  osDelay(160);

	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
  TPM1->MOD = 375000 / NOTE_G5;
  TPM1_C0V = (375000 / NOTE_G5) / 4;
  osDelay(80);

	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(80);

	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
  TPM1->MOD = 375000 / NOTE_G4;
  TPM1_C0V = (375000 / NOTE_G4) / 4;
  osDelay(160);

	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(20);
	
	TPM1->MOD = 0;
  TPM1_C0V = 0;
  osDelay(80);
}

void musicControl(void) {
	if(playFinalMusic) { // Final Music
		finalMusic();
	} else { // Normal Music
		normalMusic();
	}
}

/*----------------------------------------------------------------------------
 * Main Control Thread
 *---------------------------------------------------------------------------*/
void control_thread(void *argument) {
  for (;;) {
		osSemaphoreAcquire(manualModeSem, osWaitForever);
		switch(receivedData.cmd) {
			case forward:
			case left:
			case right:
			case reverse:
			case smallLeft:
			case smallRight:
			case smallForward:
			case stop:
				osMessageQueuePut(motorMsg, &receivedData.cmd, NULL, 0);
				break;
			case playMusic:
				playFinalMusic = 1;
				break;
			case stopMusic:
				playFinalMusic = 0;
				break;
		}
  }
}

/*----------------------------------------------------------------------------
 * Move Robot
 *---------------------------------------------------------------------------*/
void motor_thread(void *argument) {
  dataPacket rxData;
  for (;;) {
		osMessageQueueGet(motorMsg, &rxData, NULL, osWaitForever);
		switch(receivedData.cmd) {
			case forward:
				robotMovingStatus = 1;
				motorControl(forward);
				break;
			case left:
				robotMovingStatus = 1;
				motorControl(left);
				break;
			case right:
				robotMovingStatus = 1;
				motorControl(right);
				break;
			case reverse:
				robotMovingStatus = 1;
				motorControl(reverse);
				break;
			case smallLeft:
				robotMovingStatus = 1;
				motorControl(smallLeft);
			case smallRight:
				robotMovingStatus = 1;
				motorControl(smallRight);
			case smallForward:
				robotMovingStatus = 1;
				motorControl(smallForward);
			case stop:
				robotMovingStatus = 0;
				motorControl(stop);
				break;
		}
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

void autonomous_mode_thread(void *argument) {
	for(;;) {
		osSemaphoreAcquire(autonomousModeSem, osWaitForever);

		// Move forward until object detected
		robotMovingStatus = 1;
		autonomousMotorControl(forward);
		osDelay(1200);

		// Start detecting for object
		ultrasonicFlag = 1;
		osSemaphoreAcquire(autonomousStopSem, osWaitForever);
		
		// Object detected, stop
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(300);
		
		// Reverse
		robotMovingStatus = 1;
		autonomousMotorControl(reverse);
		osDelay(325);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1000);
		
		// 90 degree turn left
		robotMovingStatus = 1;
		autonomousMotorControl(left);
		osDelay(330);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1100);
		
		// Move forward
		robotMovingStatus = 1;
		autonomousMotorControl(forward);
		osDelay(500);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1100);

		// 1st 90 degree turn right
		robotMovingStatus = 1;
		autonomousMotorControl(right);
		osDelay(400);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1100);

		// Move forward
		robotMovingStatus = 1;
		autonomousMotorControl(forward);
		osDelay(1000);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1100);
		
		// 2nd 90 degree turn right
		robotMovingStatus = 1;
		autonomousMotorControl(right);
		osDelay(400);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1100);

		// Move forward
		robotMovingStatus = 1;
		autonomousMotorControl(forward);
		osDelay(1100);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1100);
		
		// 3rd 90 degree turn right
		robotMovingStatus = 1;
		autonomousMotorControl(right);
		osDelay(350);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1100);

		// Move forward
		robotMovingStatus = 1;
		autonomousMotorControl(forward);
		osDelay(1300);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1100);
		
		// 4th 90 degree turn right
		robotMovingStatus = 1;
		autonomousMotorControl(right);
		osDelay(350);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1100);

		// Move forward
		robotMovingStatus = 1;
		autonomousMotorControl(forward);
		osDelay(400);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		osDelay(1100);
		
		// 90 degree turn left
		robotMovingStatus = 1;
		autonomousMotorControl(left);
		osDelay(330);
		robotMovingStatus = 0;
		autonomousMotorControl(stop); 
		osDelay(1100);
		
		// Move forward until another object detected at the end
		robotMovingStatus = 1;
		autonomousMotorControl(forward);
		osDelay(1300);
		
		// Start detecting for object
		ultrasonicFlag = 1;
		osSemaphoreAcquire(autonomousStopSem, osWaitForever);
		
		// Object detected, stop
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		
		robotMovingStatus = 1;
		autonomousMotorControl(reverse);
		osDelay(200);
		robotMovingStatus = 0;
		autonomousMotorControl(stop);
		playFinalMusic = 1;
		osDelay(350);
	}
}

void ultrasonic_thread(void *argument) {
	for(;;) {
		// Disable LTPM counter
		TPM2_SC &= ~TPM_SC_CMOD_MASK;
		
		// TPM2_CH0 (Trigger) --> Output compare mode, clear on match
		TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK)); 
		TPM2_C0SC |= (TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK);
		
		// TPM2_CH1 (Echo) --> Input capture on rising edge
		TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
		TPM2_C1SC |= TPM_CnSC_ELSA_MASK;
		
		TPM2_CNT = 0;
		
		ultrasonicStart = 1;
		ultrasonicValue = 0;
		
		NVIC_ClearPendingIRQ(TPM2_IRQn);
		NVIC_EnableIRQ(TPM2_IRQn);
		TPM2_SC |= TPM_SC_CMOD(1);
		
		osDelay(50);
	}
}

void manual_stop_thread(void *argument) {
	for(;;) {
		osSemaphoreAcquire(manualStopSem, osWaitForever);
		robotMovingStatus = 0;
		motorControl(stop);
	}
}

int main(void) {
	// System Initialization
	SystemCoreClockUpdate();
	initUART2(BAUD_RATE);
	initMotorPins();
	initLEDPins();
	initMusicPin();
	initUltrasonic();
	
	osKernelInitialize();  // Initialize CMSIS-RTOS
	
	autonomousModeSem = osSemaphoreNew(1, 0, NULL);
	autonomousStopSem = osSemaphoreNew(1, 0, NULL);
	manualModeSem = osSemaphoreNew(1, 0, NULL);
	manualStopSem = osSemaphoreNew(1, 0, NULL);
	
	motorId = osThreadNew(motor_thread, NULL, NULL);
	controlId = osThreadNew(control_thread, NULL, NULL);
	greenLedId = osThreadNew(green_led_thread, NULL, NULL);
	redLedId = osThreadNew(red_led_thread, NULL, NULL);
	playMusicId = osThreadNew(play_music_thread, NULL, NULL);
	autonomousModeId = osThreadNew(autonomous_mode_thread, NULL, NULL);
	ultrasonicId = osThreadNew(ultrasonic_thread, NULL, &highPriority);
	manualStopId = osThreadNew(manual_stop_thread, NULL, NULL);
	
	motorMsg = osMessageQueueNew(MSG_COUNT, sizeof(dataPacket), NULL);
	
	osKernelStart();  // Start thread execution
	for (;;) {}
}
