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

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

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

void initGPIO(void) {
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

void initLED(void) {
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
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// Configure MUX settings for music pin
	PORTB->PCR[MUSIC_PIN] &= ~PORT_PCR_MUX_MASK; // Clearing Pin Control Register
	PORTB->PCR[MUSIC_PIN] |= PORT_PCR_MUX(3); // Setting Alernative 3 -> TPM1_CH0
	
	// Enable Clock to TPM1
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	// MCGFLCLK or MCGPLLCLK/2 - Select internal clock
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); 
	
	// Set MOD values for TPM1 and TPM2
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

void motorControl (int cmd) {
	int leftFrequency;
	int rightFrequency;
	switch(cmd) {
		case forward:
			leftFrequency = 5;
			rightFrequency = 5;
			TPM0_C0V = (375000 / leftFrequency) / 2;
			TPM0_C2V = (375000 / rightFrequency) / 2;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
		case left:
			leftFrequency = 20;
			rightFrequency = 5;
			TPM0_C0V = (375000 / leftFrequency) / 2;
			TPM0_C2V = (375000 / rightFrequency) / 2;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
		case right:		
			leftFrequency = 5;
			rightFrequency = 20;
			TPM0_C0V = (375000 / leftFrequency) / 2;
			TPM0_C2V = (375000 / rightFrequency) / 2;
			TPM0_C1V = 0;
			TPM0_C3V = 0;
			break;
		case reverse:
			leftFrequency = 5;
			rightFrequency = 5;
			TPM0_C0V = 0;
			TPM0_C2V = 0;
			TPM0_C1V = (375000 / leftFrequency) / 2;
			TPM0_C3V = (375000 / rightFrequency) / 2;
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
	/*
	NOTE_E5, 8, NOTE_D5, 8, NOTE_FS4, 4, NOTE_GS4, 4, 
	NOTE_CS5, 8, NOTE_B4, 8, NOTE_D4, 4, NOTE_E4, 4, 
	NOTE_B4, 8, NOTE_A4, 8, NOTE_CS4, 4, NOTE_E4, 4,
	NOTE_A4, 2
	*/
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
