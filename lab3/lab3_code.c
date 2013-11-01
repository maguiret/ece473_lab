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

#define F_CPU 16000000 // cpu speed in hertz 
#define DELAY_2_CLK do{asm("nop");asm("nop");}while(0)
#define COUNT_MAX 1023

#define CLEAR_DECODER_BITS(n) ((n) &= 0x8F)

/* Global variable to hold current displayed number */
volatile uint16_t number;

/* Array to hold active low binary encodings for base 10 digits for easy access */
uint8_t sev_seg_digits[10] = {
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
uint8_t decoder_select[6] = {
	0b00000000, //zero place
	0b00010000, //tens place
	0b00110000, //hundreds place
	0b01000000, //thousands place
	0b00010000, //center colon
	0b01110000  //hi-Z mode
};

/* Holds debounce states for each button */
uint16_t state[8] = {0,0,0,0,0,0,0,0,0};

/*****************************************************************************************
 * Function:		debounce_switch_X
 * Description:		Function takes an unsigned 8-bit integer that represents the
 * 			 pushbutton to check. Checks the state of the pin on PINX and
 * 			 returns appropriately.
 * Arguments:		button - 8 bit input number button to debounce
 * Return:		Debounced state of button
 * 			 - 0 if successful debounce
 * 			 - 1 if unsuccessful debounce
 *****************************************************************************************
 */
int8_t debounce_switch_a(uint8_t button) {
  state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
  if (state[button] == 0xF000) return 1;
  return 0;
}

/*****************************************************************************************
 * Function:		display_digits
 * Description:		Function takes a 16 bit (4 digit base 10) number and displays each
 * 			 number on the appropriate digit. 
 * Arguments:		None, number is global
 * Return:		None
 *****************************************************************************************
 */
void display_digits() 
{
	uint16_t tmp = number; //tmp variable to modify number for display
	uint8_t cur_value; //current digit value to display
	uint8_t cur_digit = 0; //current digit to display on

	//Set Register A
	DDRA = 0xFF; //output
	PORTA = 0xFF; //pullups


	/* Loop displays each base 10 digit one by one. Mods by 10 to get digit, displays
	 * encoded digit to 7-seg, divides by 10 to get next digit. Loops until cur_value
	 * is less than 1. */
	for (cur_digit = 0; cur_digit < 4; cur_digit++) {
		if (tmp >= 1) {
			cur_value = tmp % 10; //get current digit to display
			CLEAR_DECODER_BITS(PORTB); //clear portb decoder bits
			PORTA = 0xFF; //clear PORTA
			PORTB |= decoder_select[cur_digit]; //set portb decoder bits
			PORTA = sev_seg_digits[cur_value]; //display digit
		}
		_delay_loop_1(200); //delay for about arg*3 cycles
		if (tmp >= 1) {
			tmp /= 10; //get next value
			PORTB = 0x60; //switch encoder to unused bit to remove ghosting
		}
	}

}

/*****************************************************************************************
 * Function:		read_buttons
 * Description:		Function takes the button to read and calls debounce function. If
 * 			 debounce function returns true, the global number variable is
 * 			 incremented by 1<<(button number).
 * Arguments:		button - 8 bit input number button to debounce
 * Return:		None
 *****************************************************************************************
 */
void read_buttons(uint8_t button)
{
	PORTB = 0b01110000; //activate hi-z
	PORTA = 0xFF; //pullups
	DDRA = 0x00; //inputs
	DELAY_2_CLK; //delay 2 clock cycles

	if (debounce_switch_a(button))
		number += (1 << button);
}

/*****************************************************************************************
 ********************************* MAIN FUNCTION *****************************************
 *****************************************************************************************
 */
int main()
{
	uint8_t ii; //loop counter
	number = 0; //initialize number
	
	/* Initialization */
	DDRA = 0xFF; //outputs
	DDRB = 0xF0; //outputs on high nibble
	PORTA = 0xFF; //pullups
	PORTB = 0x70; //PWM low, tristate in hi-z

	while (1) {

		display_digits();

		//Read all 8 buttons
		for (ii = 0; ii < 8; ii++) 
			read_buttons(ii);
		

		//Reset number if need be
		if (number > COUNT_MAX)
			number -= COUNT_MAX;
	}
}
