/**
 * @file    ignition_interlock.c
 * @brief   An ARM Cortex M0+ breathalyzer built into an ignition interlock system
 * 			using the NXP LPC802 microcontroller (OM40000 board) for development.
 */

#include "LPC802.h"
#include "clock_config.h"
#include <stdio.h>

#define RS (4)
#define RW (17)
#define EN (16)

#define D0 (11)
#define D1 (13)
#define D2 (1)
#define D3 (10)
#define D4 (9)
#define D5 (7)
#define D6 (0)
#define D7 (8)

#define BUTTON (12)
#define LED_HEADLIGHTS	(15)

#define MRT_REPEAT (0) // Repeat mode for MRT
#define MRT_ONESHOT (1) // One shot mode for MRT
#define MRT_GFLAG0 (0) // IRQ Flag 0 (channel 0)
#define MRT_GFLAG1 (1) // IRQ Flag 1 (channel 1)
#define MRT_CHAN0 (0) // channel 0 on MRT
#define MRT_CHAN1 (1) // channel 1 on MRT
#define LED_PERIOD_TICKS (30000) // Pulses not visible to human eye.
#define INIT_COUNT_IRQ_CH0 (LED_PERIOD_TICKS/2)
#define INIT_COUNT_IRQ_CH1 (120000000)
#define INIT_PWM_DUTY_FACTOR 0.2

//prototypes
void delay(void);
void init_ADC(void);
void moveLCDCursor(void);
void setLCDNewLine(void);
void displayON(void);
void setLCDInitialMsg(void);
void setLCDFinalMsg(void);
void setLCDRetryMsg(void);
void setLCDBACMsg(int bac_val);
void setLCDBlowMsg(void);
void setLCDResultMsg(int under_limit);
void clearLCDDisplay(void);
void displayNum(int n);
void display(char c);

int volatile bac = 0;
int bac_checked = 0;
int lights_on = 0;
int is_displayed = 0;
int readings = 0;
uint32_t volatile adc_result = 0;
uint32_t volatile adc_sum;
uint32_t volatile adc_avg;
uint32_t volatile adc_buffer[10];
int press;
int adc_count;

void delay(void){
	//a simple delay function
	for (int i=0; i < 10000; i++){
		asm("NOP");
	}
}

void SysTick_Configuration(void)
{
	__disable_irq();
	NVIC_DisableIRQ(SysTick_IRQn); //turn off the SysTick interrupt.
	SysTick_Config(12000000);	// 12 MHz clock with interrupt every 1 second
	adc_count = 0;
	NVIC_EnableIRQ(SysTick_IRQn); // SysTick IRQs are on.
	__enable_irq();
}

//void SysTick_Handler(void)
//{
//	// Handle the ADC averages
//	for (int j = 0; j < 10; j++) {
//		ADC0->SEQ_CTRL[0] &= ~(1UL<<ADC_SEQ_CTRL_START_SHIFT);
//		ADC0->SEQ_CTRL[0] |= (1UL<<ADC_SEQ_CTRL_START_SHIFT);
//		adc_result = ((ADC0->DAT[2])&(ADC_DAT_RESULT_MASK));
//		adc_result = (adc_result>>ADC_DAT_RESULT_SHIFT);
//		// Add the result to the buffer
//		adc_buffer[j] = adc_result;
//	}
//}

void SysTick_Handler(void)
{
	for (int j = 0; j < 2; j++) {
		ADC0->SEQ_CTRL[0] &= ~(1UL<<ADC_SEQ_CTRL_START_SHIFT);
		ADC0->SEQ_CTRL[0] |= (1UL<<ADC_SEQ_CTRL_START_SHIFT);
		adc_result = ((ADC0->DAT[2])&(ADC_DAT_RESULT_MASK));
		adc_result = (adc_result>>ADC_DAT_RESULT_SHIFT);
		adc_buffer[adc_count] = adc_result;
	}
	adc_count = (adc_count + 1) % 10;
}

void init_ADC(void) {
	SYSCON->PDRUNCFG &=	~(SYSCON_PDRUNCFG_ADC_PD_MASK);
	SYSCON->SYSAHBCLKCTRL0 |= ( SYSCON_SYSAHBCLKCTRL0_ADC_MASK | SYSCON_SYSAHBCLKCTRL0_SWM_MASK);
	SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_ADC_RST_N_MASK);
	SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_ADC_RST_N_MASK);
	SYSCON->ADCCLKSEL &= ~(SYSCON_ADCCLKSEL_SEL_MASK);
	SYSCON->ADCCLKDIV =	1;
	SWM0->PINENABLE0 &=	~(SWM_PINENABLE0_ADC_2_MASK);
}

void MRT_Config() {
	__disable_irq(); // turn off globally
	NVIC_DisableIRQ(MRT0_IRQn);

	SYSCON->MAINCLKSEL = (0x0<<SYSCON_MAINCLKSEL_SEL_SHIFT);
	SYSCON->MAINCLKUEN &= ~(0x1);
	SYSCON->MAINCLKUEN |= 0x1;

	BOARD_BootClockFRO30M();

	SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_MRT_MASK);
	SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_MRT_RST_N_MASK);
	SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_MRT_RST_N_MASK);

	MRT0->CHANNEL[MRT_CHAN0].CTRL = (MRT_REPEAT << MRT_CHANNEL_CTRL_MODE_SHIFT | MRT_CHANNEL_CTRL_INTEN_MASK);
	MRT0->CHANNEL[MRT_CHAN0].INTVAL = INIT_COUNT_IRQ_CH0 | (MRT_CHANNEL_INTVAL_LOAD_MASK);
	MRT0->CHANNEL[MRT_CHAN1].CTRL = (MRT_REPEAT << MRT_CHANNEL_CTRL_MODE_SHIFT | MRT_CHANNEL_CTRL_INTEN_MASK);
	MRT0->CHANNEL[MRT_CHAN1].INTVAL = INIT_COUNT_IRQ_CH1 | (MRT_CHANNEL_INTVAL_LOAD_MASK);

	NVIC_EnableIRQ(MRT0_IRQn);
	__enable_irq(); // global
}

void MRT0_IRQHandler(void) {
	int static chan = 0;
	float static duty_factor = INIT_PWM_DUTY_FACTOR;
	int static pwm_state = 1;

	if (MRT0->IRQ_FLAG & (1<<MRT_GFLAG0)) {
		chan = 0;
	} else {
		chan = 1;
	}

	MRT0->CHANNEL[chan].STAT = MRT_CHANNEL_STAT_INTFLAG_MASK;
	if ((chan == 0) & (lights_on == 0)) {
		if ((bac_checked == 1) & (bac <= 8999)) {	// BAC <= 0.08 (within the legal limit)
			lights_on = 1;
			if (pwm_state == 1) // Turn on the LED if it's the ON time for PWM
			{
				GPIO->SET[0] = (1UL<<LED_HEADLIGHTS);
				MRT0->CHANNEL[MRT_CHAN0].INTVAL = (uint32_t)(LED_PERIOD_TICKS * (1-duty_factor)) & ~(MRT_CHANNEL_INTVAL_LOAD_MASK);
				pwm_state = 0;
			} else {
				GPIO->CLR[0] = (1UL<<LED_HEADLIGHTS);
				MRT0->CHANNEL[MRT_CHAN0].INTVAL = (uint32_t)(LED_PERIOD_TICKS * (duty_factor)) & ~(MRT_CHANNEL_INTVAL_LOAD_MASK);
				pwm_state = 1;
			}
		} else {	// BAC > 0.08 , car will not start
			GPIO->CLR[0] = (1UL<<LED_HEADLIGHTS);
			lights_on = 0;
		}
	} else {	// Channel 1 will execute the BAC display
		//******************
		// Calculate the ADC normalization:
		// ((aMax - aMin) / (vMax - vMin)) * (adc_avg - vMin)
		// Breath to blood alcohol conversion: 2100:1 -> 0.21
		// Internal resistance of the sensor (R0/Rs): 0.4
		// vMin = 2050, vMax = 4095
		// aMin = 0.05 mg/L, aMax = 10 mg/L
		// ((10 - 0.05) / (4095 - 2050)) * (adc_avg - 2050) * (0.4) * (0.21)
		// (199/40900) * (adc_avg - 2050) * (4/10) * (21/100)
		//******************
		if (bac_checked == 0) {
			if (adc_avg <= 2050) {	// Handle any fluctuation
				bac = 0;
				bac_checked = 1;
			} else {	// Convert air alcohol to BAC
				bac = (16716) * (adc_avg - 2050) / (40900000);
				bac_checked = 1;
			}
		}
		if (is_displayed == 0) {
			delay();
			setLCDBACMsg(bac);
			setLCDNewLine();
			if (bac <= 8999) {
				setLCDResultMsg(1);
			} else {
				setLCDResultMsg(0);
			}
			readings++;	// Increment the number of readings (max of 3)
			is_displayed = 1;
		}
	}
	return;
}

void PIN_INT0_IRQHandler(void) {
	if (PINT->IST & (1<<0)) {
		// remove the any IRQ flag for Channel 0 of GPIO INT
		PINT->IST = (1<<0);
		// DISPLAY WELCOME MESSAGE OR BLOW MESSAGE
		press++;
		//Instruct the driver to blow (if button press is not a reset)
		if (press == 1) {
			bac_checked = 0;
			clearLCDDisplay();
			setLCDBlowMsg();
			SysTick_Configuration();
			MRT_Config();
		} else {
			// For all even button presses, it will either be
			// a car shutdown or a BAC update
			if (lights_on == 1) {
				// If car is already started,
				// display final message and turn off lights
				GPIO->CLR[0] = (1UL<<LED_HEADLIGHTS);
				setLCDFinalMsg();
				lights_on = 0;
			} else {	// Car could not start (BAC too high)
				if (readings < 3) {
					clearLCDDisplay();
					setLCDRetryMsg();
					setLCDNewLine();
					setLCDBlowMsg();
					is_displayed = 0;
				} else {	// Max readings reached
					clearLCDDisplay();
					setLCDFinalMsg();
					is_displayed = 1;
				}
			}
		}
	} else {
		asm("NOP"); // Place a breakpoint here if debugging.
	}
	return;
}

int main(void) {

	// disable interrupts (global (all) and SysTick (specific))
	__disable_irq(); // turn off globally

	// HANDLE INTERRUPT AND GPIO SETUP
	NVIC_DisableIRQ(PIN_INT0_IRQn);

	SYSCON->SYSAHBCLKCTRL0 |= (SYSCON_SYSAHBCLKCTRL0_GPIO0_MASK | SYSCON_SYSAHBCLKCTRL0_GPIO_INT_MASK);
	SYSCON->PRESETCTRL0 &= ~(SYSCON_PRESETCTRL0_GPIO0_RST_N_MASK | SYSCON_PRESETCTRL0_GPIOINT_RST_N_MASK);
	SYSCON->PRESETCTRL0 |= (SYSCON_PRESETCTRL0_GPIO0_RST_N_MASK | SYSCON_PRESETCTRL0_GPIOINT_RST_N_MASK);

	// Set push button to input
	GPIO->DIRCLR[0] = (1UL<<BUTTON);

	// Set LEDs and LCD pins to outputs
	GPIO->CLR[0] = (1UL<<LED_HEADLIGHTS);
	GPIO->DIRSET[0] = (1UL<<LED_HEADLIGHTS);
	GPIO->CLR[0] = (1UL<<RS);
	GPIO->DIRSET[0] = (1UL<<RS);
	GPIO->CLR[0] = (1UL<<RW);
	GPIO->DIRSET[0] = (1UL<<RW);
	GPIO->CLR[0] = (1UL<<EN);
	GPIO->DIRSET[0] = (1UL<<EN);
	GPIO->CLR[0] = (1UL<<D0);
	GPIO->DIRSET[0] = (1UL<<D0);
	GPIO->CLR[0] = (1UL<<D1);
	GPIO->DIRSET[0] = (1UL<<D1);
	GPIO->CLR[0] = (1UL<<D2);
	GPIO->DIRSET[0] = (1UL<<D2);
	GPIO->CLR[0] = (1UL<<D3);
	GPIO->DIRSET[0] = (1UL<<D3);
	GPIO->CLR[0] = (1UL<<D4);
	GPIO->DIRSET[0] = (1UL<<D4);
	GPIO->CLR[0] = (1UL<<D5);
	GPIO->DIRSET[0] = (1UL<<D5);
	GPIO->CLR[0] = (1UL<<D6);
	GPIO->DIRSET[0] = (1UL<<D6);
	GPIO->CLR[0] = (1UL<<D7);
	GPIO->DIRSET[0] = (1UL<<D7);

	SYSCON->PINTSEL[0] = BUTTON;
	PINT->ISEL = 0x00;
	PINT->CIENR = 0b00000001;
	PINT->SIENF = 0b00000001;
	PINT->IST = 0xFF;

	NVIC_EnableIRQ(PIN_INT0_IRQn);

	//Write initial greeting to LCD
	GPIO->CLR[0] = (1UL<<RW);
	setLCDInitialMsg();

	__enable_irq(); // global

	// Initialize ADC for sensing alcohol
	init_ADC();
	ADC0->SEQ_CTRL[0] |= (1UL<<2);
	ADC0->SEQ_CTRL[0] |= (1UL<<ADC_SEQ_CTRL_TRIGPOL_SHIFT);
	ADC0->SEQ_CTRL[0] |= (1UL<<ADC_SEQ_CTRL_SEQ_ENA_SHIFT);
	ADC0->SEQ_CTRL[0] |= (1UL<<ADC_SEQ_CTRL_START_SHIFT);

	adc_avg = 0;
    while(1) {
    	adc_sum = 0;
    	for (int i = 0; i < 10; i++) {
    		adc_sum += adc_buffer[i];
    	}
    	adc_avg = adc_sum / 10;
    }
    return 0 ;
}

void displayON(void){
//	clearLCDDisplay();
	GPIO->CLR[0] = (1UL<<RS);	//interprets as command
	GPIO->SET[0] = (1UL<<EN);	//Start HIGH
	delay();

	//Display on
	GPIO->CLR[0] = (1UL<<D0);
	GPIO->CLR[0] = (1UL<<D1);
	GPIO->SET[0] = (1UL<<D2);
	GPIO->SET[0] = (1UL<<D3);
	GPIO->CLR[0] = (1UL<<D4);
	GPIO->CLR[0] = (1UL<<D5);
	GPIO->CLR[0] = (1UL<<D6);
	GPIO->CLR[0] = (1UL<<D7);

	GPIO->CLR[0] = (1UL<<EN);	//to-LOW

	// Set to 2-line mode
	GPIO->SET[0] = (1UL<<EN);	//Start HIGH
	delay();

	GPIO->CLR[0] = (1UL<<D0);
	GPIO->CLR[0] = (1UL<<D1);
	GPIO->CLR[0] = (1UL<<D2);
	GPIO->SET[0] = (1UL<<D3);
	GPIO->SET[0] = (1UL<<D4);
	GPIO->SET[0] = (1UL<<D5);
	GPIO->CLR[0] = (1UL<<D6);
	GPIO->CLR[0] = (1UL<<D7);

	GPIO->CLR[0] = (1UL<<EN);	//to-LOW
}

void clearLCDDisplay(void){
	GPIO->CLR[0] = (1UL<<RS);	//interprets as command
	GPIO->SET[0] = (1UL<<EN);	//Start HIGH
	delay();

	//Clear display
	GPIO->SET[0] = (1UL<<D0);
	GPIO->CLR[0] = (1UL<<D1);
	GPIO->CLR[0] = (1UL<<D2);
	GPIO->CLR[0] = (1UL<<D3);
	GPIO->CLR[0] = (1UL<<D4);
	GPIO->CLR[0] = (1UL<<D5);
	GPIO->CLR[0] = (1UL<<D6);
	GPIO->CLR[0] = (1UL<<D7);

	GPIO->CLR[0] = (1UL<<EN);	//to-LOW
}

void moveLCDCursor(void){
	GPIO->CLR[0] = (1UL<<RS);	//interprets as command
	GPIO->SET[0] = (1UL<<EN);	//Start HIGH
	delay();

	//move cursor right
	GPIO->CLR[0] = (1UL<<D0);
	GPIO->CLR[0] = (1UL<<D1);
	GPIO->SET[0] = (1UL<<D2);
	GPIO->CLR[0] = (1UL<<D3);
	GPIO->SET[0] = (1UL<<D4);
	GPIO->CLR[0] = (1UL<<D5);
	GPIO->CLR[0] = (1UL<<D6);
	GPIO->CLR[0] = (1UL<<D7);

	GPIO->CLR[0] = (1UL<<EN);	//to-LOW
}

void setLCDNewLine(void) {
	GPIO->CLR[0] = (1UL<<RS);	//interprets as command
	GPIO->SET[0] = (1UL<<EN);	//Start HIGH
	delay();

	// Move cursor to the 2nd line
	GPIO->CLR[0] = (1UL<<D0);
	GPIO->CLR[0] = (1UL<<D1);
	GPIO->CLR[0] = (1UL<<D2);
	GPIO->CLR[0] = (1UL<<D3);
	GPIO->CLR[0] = (1UL<<D4);
	GPIO->CLR[0] = (1UL<<D5);
	GPIO->SET[0] = (1UL<<D6);
	GPIO->SET[0] = (1UL<<D7);

	GPIO->CLR[0] = (1UL<<EN);	//to-LOW
}

void setLCDInitialMsg(void){
	//display initial message "HELLO <DRIVER NAME>"
	clearLCDDisplay();
	displayON();

	display('H');
	display('E');
	display('L');
	display('L');
	display('O');
	moveLCDCursor();
	display('J');
	display('U');
	display('L');
	display('I');
	display('A');
	display('!');

	setLCDNewLine();
	display('P');
	display('U');
	display('S');
	display('H');
	moveLCDCursor();
	display('T');
	display('O');
	moveLCDCursor();
	display('S');
	display('T');
	display('A');
	display('R');
	display('T');
}

void setLCDBlowMsg(void){
	//display ignition message "BLOW 5 TIMES"
	display('B');
	display('L');
	display('O');
	display('W');
	moveLCDCursor();
	displayNum(5);
	moveLCDCursor();
	display('T');
	display('I');
	display('M');
	display('E');
	display('S');
	display('.');
	display('.');
	display('.');
}

void setLCDFinalMsg(void){
	//display initial message "GOODBYE!"
	clearLCDDisplay();

	if (readings < 3) {
		display('Y');
		display('O');
		display('U');
		moveLCDCursor();
		display('H');
		display('A');
		display('V');
		display('E');
		moveLCDCursor();
		display('A');
		display('R');
		display('R');
		display('I');
		display('V');
		display('E');
		display('D');

		setLCDNewLine();

		display('S');
		display('A');
		display('F');
		display('E');
		display('L');
		display('Y');
		display('.');
		moveLCDCursor();
		display('G');
		display('O');
		display('O');
		display('D');
		display('B');
		display('Y');
		display('E');
		display('!');
	} else {
		display('C');
		display('A');
		display('L');
		display('L');
		moveLCDCursor();
		display('A');
		display('N');
		moveLCDCursor();
		display('U');
		display('B');
		display('E');
		display('R');
		moveLCDCursor();
		display('O');
		display('R');

		setLCDNewLine();
		display('#');
		display('T');
		display('A');
		display('X');
		display('I');
		moveLCDCursor();
		display('(');
		display('#');
		displayNum(8);
		displayNum(2);
		displayNum(9);
		displayNum(4);
		display(')');
		moveLCDCursor();
	}
}

void setLCDRetryMsg(void) {
//	clearLCDDisplay();
	display('T');
	display('R');
	display('Y');
	moveLCDCursor();
	display('A');
	display('G');
	display('A');
	display('I');
	display('N');
}

void setLCDBACMsg(int bac_val){
	//display the message "BAC LEVEL: "
	clearLCDDisplay();

	display('B');
	display('A');
	display('C');
	moveLCDCursor();
	display('L');
	display('E');
	display('V');
	display('E');
	display('L');
	display(':');
	moveLCDCursor();

	if (bac_val < 10) {	// If reading is < 0.01 % BAC
		displayNum(0);
		display('.');
		displayNum(0);
		displayNum(0);
	} else {	// Display the BAC calculation
		int n;
		displayNum(0);
		display('.');
		n = bac / 10000;
		displayNum(n);
		n = (bac % 10000) / 1000;
		displayNum(n);
	}
	display('%');
	///////////////
}

void setLCDResultMsg(int under_limit) {
	if (under_limit == 1) {
		display('D');
		display('R');
		display('I');
		display('V');
		display('E');
		moveLCDCursor();
		display('S');
		display('A');
		display('F');
		display('E');
		display('!');
	} else {
		display('T');
		display('O');
		display('O');
		moveLCDCursor();
		display('H');
		display('I');
		display('G');
		display('H');
	}
}

void displayNum(int n) {
	GPIO->SET[0] = (1UL<<RS);	//set HIGH to interpret digital pins as data
	GPIO->SET[0] = (1UL<<EN);	//begin HIGH to load data bits into D0-D7
	delay();

	if (n == 0) {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (n == 1) {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (n == 2) {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (n == 3) {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (n == 4) {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (n == 5) {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (n == 6) {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (n == 7) {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (n == 8) {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (n == 9) {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	}

	GPIO->CLR[0] = (1UL<<EN);	//end LOW to load data bits

}

void display(char c) {
	GPIO->SET[0] = (1UL<<RS);	//set HIGH to interpret digital pins as data
	GPIO->SET[0] = (1UL<<EN);	//begin HIGH to load data bits into D0-D7
	delay();

	if (c == 'A') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'B') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'C') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'D') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'E') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'F') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'G') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'H') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'I') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'J') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	}
	else if (c == 'L') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'M') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'N') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'O') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'P') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	}
	else if (c == 'R') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'S') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'T') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'U') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'V') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'W') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'X') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == 'Y') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
		GPIO->CLR[0] = (1UL<<D5);
		GPIO->SET[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	}
	else if (c == '.') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == ':') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->SET[0] = (1UL<<D4);
//		GPIO->CLR[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == '!') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == '%') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->SET[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == '#') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->SET[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->CLR[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == ')') {
		GPIO->SET[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	} else if (c == '(') {
		GPIO->CLR[0] = (1UL<<D0);
		GPIO->CLR[0] = (1UL<<D1);
		GPIO->CLR[0] = (1UL<<D2);
		GPIO->SET[0] = (1UL<<D3);
		GPIO->CLR[0] = (1UL<<D4);
		GPIO->SET[0] = (1UL<<D5);
		GPIO->CLR[0] = (1UL<<D6);
		GPIO->CLR[0] = (1UL<<D7);
	}

	GPIO->CLR[0] = (1UL<<EN);	//end LOW to load data bits
}
