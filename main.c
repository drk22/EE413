//******************************************************************************
/*
 *   Filename:      MSP430 Serial Data.c
 *   Date:          2012/03/03
 *   File Version:  1.0
 *
 *   Author:        Michael Kremlicka
 *   Company:
 *
 *************************************************************************
 *
 *   Architecture:  MSP430
 *   Processor:     MSP430G2553
 *   Compiler:      Code Composer Studio  Version: 5.1.1.00031
 *
 *************************************************************************
 *
 *   Files required: none
 *
 *************************************************************************
 *
 *   Description:
 *
 *   	ACLK = n/a, MCLK = SMCLK = CALB_1MHZ, ADC10CLK = ADC10OSC/4
 *
 *   	ADC sample is made on A10 with reference to internal
 *  			1.5V Vref. Temperature in oC stored in IntDegC.
 *  		ADC sample is made on A11 with reference to internal
 *  			2.5V Vref. MPS430 power supply voltage is mV stored in
 *  			IntVoltmV.
 *   	ADC sampling/converting/ is started by
 *   		Timer_A.OUT1 (in every 1 second an ADC measuring is done)
 *
 *   	HW UART is used to send measured data to PC via SERIAL PORT
 *   		Baud Rate: 9000 Baud
 *
 *		LEDs indicating ADC sampling
 *
 *                MSP430G2x33/G2x53
 *             -----------------
 *         /|\|              XIN|-
 *          | |                 |
 *          --|RST          XOUT|-
 *            |                 |
 *            |A10              |
 *
 *
 *******************************************************************************
 */

#include  "msp430g2553.h"
#include <stdio.h>
#include <stdlib.h>;

#define LED_RED	BIT0
#define LED_GRE BIT6
#define LEDs 	LED_RED + LED_GRE

volatile long temp;
volatile long volt;
long IntVoltmV;
long IntDegC;
long TempHold = 0;
long threshTempHeat;
long threshTempCool;
const int threshold = 5; // tolerance for heating/ cooling range

volatile enum {
	MEASTEMP, MEASVDD
} measmode;

//function prototypes
void UART_puts(char * s);
void UART_outdec(long data, unsigned char ndigits);
void pushbutton_interrupt_int(void);

volatile int operation_state = 0;
volatile int temp_range;


void main(void) {
	//const int threshold = 5; // tolerance for heating/ cooling range
	WDTCTL = WDTPW + WDTHOLD; // Stop WDT
	pushbutton_interrupt_int();
	//Initialize DCO
	if (CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xFF) {
		while (1)
			; // If calibration constants erased
			  // do not load, trap CPU!!
	}
	BCSCTL1 = CALBC1_1MHZ; // Set DCO
	DCOCTL = CALDCO_1MHZ;

	//Initialize ADC
	measmode = MEASTEMP;
	ADC10CTL1 = INCH_10 + ADC10DIV_3 + SHS_1; // Temp Sensor ADC10CLK/4
	ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
	TACCR0 = 30; // Delay to allow Ref to settle
	TACCTL0 |= CCIE; // Compare-mode interrupt
	TACTL = TASSEL_2 + MC_1; // TACLK = SMCLK, Up mode
	__bis_SR_register(CPUOFF + GIE);
	// LPM0, TA0_ISR will force exit
	TACCTL0 &= ~CCIE; // Disable timer Interrupt

	BCSCTL3 |= LFXT1S_2; // ACLK = VLO
	TACCR0 = 12000; //
	TACCTL1 = OUTMOD_3; // TACCR1 set/reset
	TACCR1 = 6000; // TACCR1 PWM Duty Cycle
	//TACCTL0 |= CCIE;                        // Compare-mode interrupt
	TACTL = TASSEL_1 + MC_1; // TACLK = ACLK, Up mode

	//Initialize HW UART
	P1SEL = BIT1 + BIT2; // P1.1 = RXD, P1.2=TXD
	P1SEL2 = BIT1 + BIT2; // Secondary peripheral module function is selected.
	UCA0CTL1 |= UCSSEL_2; // SMCLK
	UCA0BR0 = 104; // 1MHz 9600
	UCA0BR1 = 0; // 1MHz 9600
	UCA0MCTL = UCBRS0; // Modulation UCBRSx = 1

	UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
	IE2 |= 0x1;
	// Initialize LED_RED LED_GRE
	P1DIR |= LED_RED + LED_GRE;

	ADC10CTL0 |= ENC;
	while (1) {
		__bis_SR_register(CPUOFF + GIE);
		// LPM0 with interrupts enabled

		switch (measmode) {
		case MEASTEMP:
			//P1OUT |= LED_RED;
			temp = ADC10MEM;
			IntDegC = ((temp - 673) * 4225) / 1024;
			//UART_outdec(IntDegC,1);
			//UART_puts("C");
			measmode = MEASVDD;
			break;
		case MEASVDD:
			//P1OUT |= LED_GRE;
			volt = ADC10MEM;
			IntVoltmV = volt * 5000 / 1024;
			//UART_outdec(IntVoltmV,0);
			//UART_puts("mV");
			measmode = MEASTEMP;
			break;
		}
		if (operation_state){

			if(IntDegC > TempHold + threshold){

				P1OUT |= BIT6;
				P1OUT &= ~BIT0;
			}

			if(IntDegC < TempHold - threshold){

							P1OUT |= BIT0;
							P1OUT &= ~BIT6;
			}
/*		switch (temp_range) { //sets for cooling mode

		case 1:
			threshTempHigh = TempHold + threshold;
			threshTempLow = TempHold - threshold;

			if(TempHold < threshTempLow){
				P1OUT |= BIT0; //Red LED to signify Heat mode
			}
			else if(TempHold > threshTempHigh){
				P1OUT &= ~ BIT0;
				P1OUT |= BIT6; 	//Red LED off, Green LED on to signify cooling mode
			}
			break;
		}
	*/
			}
		//UART_puts("\n\r");

		//	P1OUT &= ~(LED_RED + LED_GRE);
		__no_operation(); // SET BREAKPOINT HERE
	}

}

//-------------------------------------------------
// function definitions
//-------------------------------------------------

void UART_puts(char * s) {
	while (*s) {
		while (!(IFG2 & UCA0TXIFG))
			; // USCI_A0 TX buffer ready?
		UCA0TXBUF = *s++;
	}
}

void UART_outdec(long data, unsigned char ndigits) {
	unsigned char sign, s[6];
	unsigned int i;
	sign = ' ';
	if (data < 0) {
		sign = '-';
		data = -data;
	}
	i = 0;
	do {
		s[i++] = data % 10 + '0';
		if (i == ndigits) {
			s[i++] = '.';
		}
	} while ((data /= 10) > 0);
	s[i] = sign;
//	while (i<5){
	//	s[i++] = 'x';
//	}
	do {
		while (!(IFG2 & UCA0TXIFG))
			;
		UCA0TXBUF = s[i];
	} while (i--);
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void) {
	__bic_SR_register_on_exit(CPUOFF);
	// Clear CPUOFF bit from 0(SR)
	ADC10CTL0 &= ~ENC; // ADC10 disable
	switch (measmode) {
	case MEASTEMP:
		//temp = ADC10MEM;
		ADC10CTL1 = INCH_11 + ADC10DIV_3 + SHS_1;
		ADC10CTL0 |= REF2_5V;
		break;
	case MEASVDD:
		//volt = ADC10MEM;
		ADC10CTL1 = INCH_10 + ADC10DIV_3 + SHS_1;
		ADC10CTL0 &= ~REF2_5V;
		break;
	}
	ADC10CTL0 |= ENC; //
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TA0_ISR(void) {
	//TACTL = 0;                                // Clear Timer_A control registers
	__bic_SR_register_on_exit(CPUOFF);
	// Clear CPUOFF bit from 0(SR)
	//ADC10CTL0 = SREF_1 + ADC10SHT_3 + REFON + ADC10ON + ADC10IE;
	//ADC10CTL0 |= ENC;
}

#pragma vector=PORT1_VECTOR
__interrupt void P13_ISR(void) {

	operation_state = 1;
	TempHold = IntDegC;

	temp_range = 0;

	UART_outdec(IntDegC, 1);
	UART_puts("C");
	UART_puts("\n");

	IFG1 &= ~OFIFG;
	//IFG1 &= ~P3;
	P1OUT ^= BIT0;
	P1IFG ^= BIT3;
}

void pushbutton_interrupt_int(void) {

	P1SEL |= BIT3; // P1SEL = 0X2
	P1DIR &= BIT3;
	P1IE |= BIT3;

}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {
	char input = UCA0RXBUF;
	char min = -40;
	char max = 85;
	//delay();
	if (input == 't') {
		UART_outdec(IntDegC, 1);
		UART_puts("C");
		UART_puts("\n");

	} //if (input>=min && input<=max){
		//take user entered temperature and set as the reference
//		TempHold = input;
//	}
	else if (input == 'c') {
		P1OUT ^= BIT6;
		P1OUT &= ~BIT0;
	} else if (input == 'h') {
		P1OUT &= ~BIT6;
		P1OUT ^= BIT0;
	} else if (input == 'o') {
		operation_state = 0;
	}

	else {
		UART_puts("Invalid Input");
		UART_puts("\n\r");
	}
	IFG2 &= ~UCA0RXIFG;

}






