/*
 * Created by: Jacob Branaugh
 * Created on: 10/22/2013 14:30
 *
 * Code for ECE 473 Lab.
 * 
 ****************************HARDWARE SETUP***********************************************
 * - PORTA is connected to the segments of the LED display. and to the pushbuttons.
 * - PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
 * - PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
 * - PORTB bit 7 goes to the PWM transistor base.
 * - PORTB bit 0 goes to shift load on the bar graph display
 * - PORTB bit 1 goes to the clock inputs of the bar graph and encoder boards
 * - PORTB bit 2 goes to the serial in of the bar graph display
 * - PORTB bit 3 goes to the serial out of the encoder board
 *****************************************************************************************
 *
 ****************************LAB 2 SPECIFICS**********************************************
 * - Pull PWM pin PORTB 7 low
 * - LED segments are active low
 * - Buttons are active low
 * - EN_N tied low, EN tied high on LED board
 * - COM_EN on button board tied to DEC7 on LED board
 *   - Tristate in HI-Z when DEC7 high (PORTB |= 0b01110000)
 * - COM_LVL on button board tied low
 *****************************************************************************************
 *
 ****************************LAB 3 SPECIFICS**********************************************
 * - 
 *****************************************************************************************
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd_functions.h"

#define F_CPU 16000000 // cpu speed in hertz 

#define SS PB0
#define SCK PB1
#define MOSI PB2
#define COUNT_MAX 1023

/* 2 cycle delay */
#define DELAY_CLK do{asm("nop");asm("nop");}while(0)

/* Global variable to hold current displayed number */
volatile uint16_t number;

/* Global variable to hold mode determined by push buttons */
volatile uint8_t pushbutton_mode = 0x00;

/* Sets the step size for the encoder counter */
volatile uint8_t step_size = 1;

/* Variables to hold prior states of encoders */
volatile uint8_t encoder1_prev_a = 0x00;
volatile uint8_t encoder1_prev_b = 0x00;
volatile uint8_t encoder2_prev_a = 0x00;
volatile uint8_t encoder2_prev_b = 0x00;

/* Array to hold active low binary encodings for base 10 digits for easy access */
volatile uint8_t sev_seg_digits[10] = {
	0b11000000, //0
	0b11111001, //1
	0b10100100, //2
	0b10110000, //3
	0b10011001, //4
	0b10010010, //5
	0b10000011, //6
	0b11111000, //7
	0b10000000, //8
	0b10011000  //9
};

/* Array to hold binary encodings for PORTB digit select, where the place in the array
 * corresponds to the digit place */
volatile uint8_t decoder_select[6] = {
	0b00000000, //zero place
	0b00010000, //tens place
	0b00110000, //hundreds place
	0b01000000, //thousands place
	0b00010000, //center colon
	0b01110000  //hi-Z mode
};

/* Holds debounce states for each button */
volatile uint16_t state[8] = {0,0,0,0,0,0,0,0};


/*****************************************************************************************
 * Function:		TCNT0_init
 * Description:		Initializes timer/counter 0
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
void TCNT0_init() 
{
	TIMSK |= (1<<TOIE0); //enable timer/counter0 overflow interrupt
	TCCR0 |= (1<<CS01); //normal mode, prescale by 8
}

/*****************************************************************************************
 * Function:		SPI_init
 * Description:		Initializes SPI on PORTB in master mode
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
void SPI_init()
{
	DDRB |= ((1 << SS) | //turns on slave select
		 (1 << MOSI) | //sets MOSI (master out, slave in)
		 (1 << SCK));  //sets master clock output
	SPCR |= ((1 << SPE) |  //enables SPI
		 (1 << MSTR));  //sets master SPI mode
	SPSR |= (1 << SPI2X);  //sets a clock/2 prescalar
}

/*****************************************************************************************
 * Function:		debounce_switch_X
 * Description:		Function takes an unsigned 8-bit integer that represents the
 * 			 pushbutton to check. Checks the state of the pin on PINX and
 * 			 returns appropriately.
 * Arguments:		button - 8 bit input number button to debounce
 * Return:		Debounced state of button
 * 			 - 0 if successful debounce
 * 			 - 1 if unsuccessful debounce
 ****************************************************************************************/
int8_t debounce_switch_a(uint8_t button) 
{
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
	if (state[button] == 0xF000) return 1;
	return 0;
}

/*****************************************************************************************
 * Function:		button_mode_toggle
 * Description:		Function takes button number and toggles corresponding bits in
 * 			 mode variable
 * Arguments:		Button number
 * Return:		Modifies global mode variable
 ****************************************************************************************/
void button_mode_toggle(uint8_t button)
{
	if (button == 0)
		pushbutton_mode ^= 0x01; //toggle first bit
	else if (button == 1)
		pushbutton_mode ^= 0x02; //toggle second bit
}

/*****************************************************************************************
 * Function:		display_digits
 * Description:		Function takes a 16 bit (4 digit base 10) number and displays each
 * 			 number on the appropriate digit. 
 * Arguments:		None, number is global
 * Return:		None
 ****************************************************************************************/
void display_digits() 
{
	uint16_t tmp = number; //tmp variable to modify number for display
	uint8_t cur_value; //current digit value to display
	uint8_t cur_digit = 0; //current digit to display on

	//Set Register A
	PORTA = 0xFF; //pullups
	DDRA = 0xFF; //output
	DELAY_CLK;

	/* Loop displays each base 10 digit one by one. Mods by 10 to get digit, displays
	 * encoded digit to 7-seg, divides by 10 to get next digit. Loops until cur_value
	 * is less than 1. */
	for (cur_digit = 0; cur_digit < 4; cur_digit++) {

		PORTA = 0xFF; //clear PORTA
		DELAY_CLK;
		PORTB &= 0x8F; //clear decoder bits
		DELAY_CLK;
		PORTB |= decoder_select[cur_digit]; //set portb decoder bits

		/* Display when number is 0 */
		if (tmp < 1 && cur_digit == 0) {
			PORTA = sev_seg_digits[0]; //display digit
		}

		/* Display current digit */
		if (tmp >= 1) {
			cur_value = tmp % 10; //get current digit to display
			PORTA = sev_seg_digits[cur_value]; //display digit
		}

		_delay_loop_1(200); //delay for about arg*3 cycles

		/* Get next digit if possible */
		if (tmp >= 1) {
			tmp /= 10; //get next value
		}
	}
	//take this out
	//PORTB |= 0x70;
	//PORTA = 0xFF;
	//DDRA = 0x00;
	//DELAY_CLK; //let everything settle
	//take this out
}

/*****************************************************************************************
 * Function:		read_buttons
 * Description:		Function takes the button to read and calls debounce function. If
 * 			 debounce function returns true, the global number variable is
 * 			 incremented by 1<<(button number).
 * Arguments:		button - 8 bit input number button to debounce
 * Return:		None
 ****************************************************************************************/
void read_buttons()
{
	uint8_t button;

	PORTB &= 0x8F;
	PORTB |= 0x50; //set decoder to output 5 to turn off transistors
	DELAY_CLK;

	PORTB |= 0x70; //activate hi-z, leave everything else
	PORTA = 0xFF; //pullups
	DDRA = 0x00; //inputs
	DELAY_CLK; //let everything settle

	/* check each button with switch debouncing */
	for (button = 0; button < 8; button++)
		if (debounce_switch_a(button))
			button_mode_toggle(button);
}

/*****************************************************************************************
 * Function:		read_encoder
 * Description:		Checks current state of encoder, compares against previous state
 * 			 to determine whether encoder is being turned clockwise or
 * 			 counterclockwise.
 * Arguments:		Integer number 1 or 2, representing one of the encoders
 * Return:		0 for clockwise turn, 1 for counterclockwise turn
 ****************************************************************************************/
uint8_t read_encoder(uint8_t encoder)
{
	uint8_t ret = -1;
	uint8_t encoder_cur_a, encoder_cur_b; //holds current states
	if (encoder == 1) {
		encoder_cur_a = SPDR & 0x01; //gets first bit read from SPI
		encoder_cur_b = (SPDR & 0x02) >> 1; //gets second bit
		if (encoder_cur_a == 1 && encoder_cur_b == 1) { //if encoder turned
			if (encoder1_prev_a == 1 && encoder1_prev_b == 0) //clockwise
				ret = 0;
			else if (encoder1_prev_a == 0 && encoder1_prev_b == 1) //counter
				ret = 1;
		}
		/* Stores values for later */
		encoder1_prev_a = encoder_cur_a;
		encoder1_prev_b = encoder_cur_b;
	} else if (encoder == 2) {
		encoder_cur_a = (SPDR & 0x04) >> 2; //gets third bit read from SPI
		encoder_cur_b = (SPDR & 0x08) >> 3; //gets fourth bit
		if (encoder_cur_a == 1 && encoder_cur_b == 1) { //if encoder turned
			if (encoder2_prev_a == 1 && encoder2_prev_b == 0) //clockwise
				ret = 0;
			else if (encoder2_prev_a == 0 && encoder2_prev_b == 1) //counter
				ret = 1;
		}
		/* Stores values for later */
		encoder2_prev_a = encoder_cur_a;
		encoder2_prev_b = encoder_cur_b;
	}

	return ret;
}

///*****************************************************************************************
// * Function:		Interrupt Service Routine for Timer/Counter 0
// * Description:		Timer runs in asynchronous mode off of crystal oscillator. On
// * 			 overflow, a counter is incremented. When the counter reaches 128,
// * 			 a second of time has passed, and the seconds variable is
// * 			 incremented and the counter is set to zero. If seconds has
// * 			 reached the end of a day, it is also reset to zero. The buttons
// * 			 are read every interrupt.
// * Arguments:		None
// * Return:		None
// ****************************************************************************************/
//ISR(TIMER0_OVF_vect)
//{
//	read_buttons();
//
//	/* Sets the counter step size based on button mode
//	 * steps by 1 (default) if neither pressed
//	 * steps by 2 if button 1 pressed
//	 * steps by 4 if button 2 pressed
//	 * doesn't count if both pressed */
//	if (pushbutton_mode == 0x00)
//		step_size = 0x01;
//	else if (pushbutton_mode == 0x01)
//		step_size = 0x02;
//	else if (pushbutton_mode == 0x02)
//		step_size = 0x04;
//	else if (pushbutton_mode == 0x03)
//		step_size = 0x00;
//
//	/* Sets leds on bar graph display */
//	SPDR = pushbutton_mode; //sets value of SPI data register to mode value
//	while(bit_is_clear(SPSR, SPIF)); //waits for serial transmission to complete
//	PORTB |= 0x70;
//	PORTB &= 0xEF; //toggle bar graph regclk
//	PORTB |= 0x10;
//
//	/* Check both encoders for rotation */
//	PORTB |= 0x01; //toggle shift load on encoder board
//	SPDR = 0x00; //write a zero for filler purposes
//	while(bit_is_clear(SPSR, SPIF)); //wait for write to finish
//	PORTB &= 0xFE; //clear shift load bit on encoder board
//
//	uint8_t check_1 = read_encoder(1);
//	uint8_t check_2 = read_encoder(2);
//
//	/* If a clockwise turn was made, increment count */
//	if (check_1 == 0 || check_2 == 0)
//		number += step_size;
//
//	/* If a counterclockwise turn was made, decrement count */
//	if (check_1 == 1 || check_2 == 1)
//		number -= step_size;
//
//	/* Ensure number is always between 0 and COUNT_MAX */
//	number %= COUNT_MAX + 1;
//	
//	display_digits();
//}

/*****************************************************************************************
 * Function:		Interrupt Service Routine for Timer/Counter 0
 * Description:		Timer runs in asynchronous mode off of crystal oscillator. On
 * 			 overflow, a counter is incremented. When the counter reaches 128,
 * 			 a second of time has passed, and the seconds variable is
 * 			 incremented and the counter is set to zero. If seconds has
 * 			 reached the end of a day, it is also reset to zero. The buttons
 * 			 are read every interrupt.
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
ISR(TIMER0_OVF_vect)
{
	uint8_t old_PORTA = PORTA;
	uint8_t old_PORTB = PORTB;
	uint8_t old_DDRA = DDRA;

	read_buttons();

	PORTA = old_PORTA;
	PORTB = old_PORTB;
	DDRA = old_DDRA;
	DELAY_CLK;

	/* Sets the counter step size based on button mode
	 * steps by 1 (default) if neither pressed
	 * steps by 2 if button 1 pressed
	 * steps by 4 if button 2 pressed
	 * doesn't count if both pressed */
	if (pushbutton_mode == 0x00)
		step_size = 0x01;
	else if (pushbutton_mode == 0x01)
		step_size = 0x02;
	else if (pushbutton_mode == 0x02)
		step_size = 0x04;
	else if (pushbutton_mode == 0x03)
		step_size = 0x00;

	/* Sets leds on bar graph display */
	SPDR = pushbutton_mode; //sets value of SPI data register to mode value
	while(bit_is_clear(SPSR, SPIF)); //waits for serial transmission to complete
	PORTB |= 0x70;
	PORTB &= 0xEF; //toggle bar graph regclk
	PORTB |= 0x10;

	/* Check both encoders for rotation */
	PORTB |= 0x01; //toggle shift load on encoder board
	SPDR = 0x00; //write a zero for filler purposes
	while(bit_is_clear(SPSR, SPIF)); //wait for write to finish
	PORTB &= 0xFE; //clear shift load bit on encoder board

	uint8_t check_1 = read_encoder(1);
	uint8_t check_2 = read_encoder(2);

	/* If a clockwise turn was made, increment count */
	if (check_1 == 0 || check_2 == 0)
		number += step_size;

	/* If a counterclockwise turn was made, decrement count */
	if (check_1 == 1 || check_2 == 1)
		number -= step_size;

	/* Ensure number is always between 0 and COUNT_MAX */
	number %= COUNT_MAX + 1;

	PORTB = old_PORTB;
	DELAY_CLK;
}

/*****************************************************************************************
 * Function:		Interrupt Service Routine for Timer/Counter 1
 * Description:		On TCNT0 overflow, buttons are checked to set mode, mode is
 * 			 displayed on the bar graph display, the encoders are checked, and
 * 			 the global count is incremented or decremented based on whether
 * 			 the encoder was turned clockwise or counter clockwise
 * 			 respectively
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
ISR(TIMER1_OVF_vect)
{
}

/*****************************************************************************************
 * Function:		Interrupt Service Routine for Timer/Counter 2
 * Description:		On TCNT0 overflow, buttons are checked to set mode, mode is
 * 			 displayed on the bar graph display, the encoders are checked, and
 * 			 the global count is incremented or decremented based on whether
 * 			 the encoder was turned clockwise or counter clockwise
 * 			 respectively
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
ISR(TIMER2_COMP_vect)
{
}

/*****************************************************************************************
 * Function:		Interrupt Service Routine for Timer/Counter 3
 * Description:		On TCNT0 overflow, buttons are checked to set mode, mode is
 * 			 displayed on the bar graph display, the encoders are checked, and
 * 			 the global count is incremented or decremented based on whether
 * 			 the encoder was turned clockwise or counter clockwise
 * 			 respectively
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
ISR(TIMER3_COMPA_vect)
{
}


/*****************************************************************************************
 ********************************* MAIN FUNCTION *****************************************
 *****************************************************************************************
 */
int main()
{
	number = 0; //initialize number
	
	/* Initialization */
	DDRA = 0xFF; //outputs
	DDRB = 0xF0; //outputs on high nibble
	TCNT0_init(); //initialize the 8-bit timer counter register
	SPI_init(); //initialize SPI master on PORTB 1-3

	/* enable interrupts */
	sei();

	while (1) {
		display_digits();
	}
}
