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
 */

#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define DELAY 10

#define CENTER_COLON 4
#define HI_Z 1
#define CLEAR_DECODER_BITS 0b10001111

volatile uint16_t number;

uint8_t sev_seg_digits[10] = {
	0b11000000, //0 (active low)
	0b11111001, //1 (active low)
	0b10100100, //2 (active low)
	0b10110000, //3 (active low)
	0b10011001, //4 (active low)
	0b10010010, //5 (active low)
	0b10000011, //6 (active low)
	0b11111000, //7 (active low)
	0b10000000, //8 (active low)
	0b10011000  //9 (active low)
};

uint8_t decoder_select[6] = {
	0b00000000, //right most digit
	0b00010000,
	0b00110000,
	0b01000000, //left most digit
	0b00010000, //center colon
	0b01110000  //hi-Z mode
};

//holds debounce states for each button
uint16_t state[8] = {0,0,0,0,0,0,0,0,0};

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch_a(uint8_t button) {
  //static uint16_t state = 0; //holds present state
  state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
  if (state[button] == 0xF000) return 1;
  return 0;
}

int8_t debounce_switch_d(uint8_t button) {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, button)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}

/*****************************************************************************************
 * Function:		display_digits
 * Description:		Function takes a 16 bit (4 digit base 10) number and displays each
 * 			number on the appropriate digit. 
 * Arguments:		num - 16 bit input number
 * Return:		None
 *****************************************************************************************
 */
void display_digits() 
{
	uint16_t tmp = number;
	uint8_t cur_value;
	uint8_t cur_digit = 0;
	//uint8_t num_digits = 0;

	do {
		cur_value = tmp % 10; //get current digit to display
		PORTB &= CLEAR_DECODER_BITS; //clear portb decoder bits
		PORTB |= decoder_select[cur_digit]; //set portb decoder bits
		PORTA = 0xFF;
		PORTA = sev_seg_digits[cur_value]; //display digit
		_delay_ms(DELAY);
		cur_digit++; //next digit
		tmp /= 10; //get next value
	} while (tmp >= 1);
}

/*****************************************************************************************
 * Function:		read_buttons
 * Description:		Function takes the address of the current 16 bit number in the 
 * 			count, reads the pushbuttons connected to PORTA, and increments
 * 			the number appropriately.
 * Arguments:		*num - pointer to 16 bit integer
 * Return:		None (modified num by reference)
 *****************************************************************************************
 */
void read_buttons(uint8_t button)
{
	uint8_t ii;
	PORTB = 0b01110000; //activate hi-z
	DDRA = 0x00; //inputs
	PORTA = 0xFF; //pullups

	//for (ii = 0; ii < 8; ii++) {
	//	//if (debounce_switch_a(ii))
	//	if (debounce_switch_d(ii))
	//		number += (1 << ii);
	//}
	if (debounce_switch_a(button))
		number += (1 << button);
	


	//if (debounce_switch_a(0))
	//	*num += 1;
	//if (debounce_switch_a(0))
	//	*num += 1;
	//if (debounce_switch_a(1))
	//	*num += 2;
	//if (debounce_switch_a(2))
	//	*num += 4;
	//if (debounce_switch_a(3))
	//	*num += 8;
	//if (debounce_switch_a(4))
	//	*num += 16;
	//if (debounce_switch_a(5))
	//	*num += 32;
	//if (debounce_switch_a(6))
	//	*num += 64;
	//if (debounce_switch_a(7))
	//	*num += 128;

}

/*****************************************************************************************
 ********************************* MAIN FUNCTION *****************************************
 *****************************************************************************************
 */
int main()
{
	uint8_t ii;
	number = 0;
	
	/* Initialization */
	DDRA = 0xFF; //outputs
	DDRB = 0xF0; //outputs on high nibble
	PORTA = 0xFF; //pullups
	PORTB = 0x70; //PWM low, tristate in hi-z
	DDRD = 0x00;
	PORTD = 0xFF;

	while (1) {

		//DISPLAY
		DDRA = 0xFF; //output
		PORTA = 0xFF; //pullups
		display_digits();
		PORTB = 0x60; //switch encoder output to unused bit to remove ghosting

		//READ
		for (ii = 0; ii < 8; ii++) {
			read_buttons(ii);
		}
		//read_buttons(0);

		if (number > 1023)
			number = 1;
		//END
		
		//if (debounce_switch()) {
		//	//counter = (counter < 8 ? counter++ : 0);
		//	if (counter > 8) {
		//		counter = 0;
		//	} else {
		//		counter++;
		//	}
		//}
		//if (counter == 8)
		//	PORTA = 0x00;
		//else
		//	PORTA = 0xFF & ~(1 << counter);

		//PORTB = 0x00;
		//_delay_ms(DELAY);
		//PORTB = 0x10;
		//_delay_ms(DELAY);
		//PORTB = 0x20;
		//_delay_ms(DELAY);
		//PORTB = 0x30;
		//_delay_ms(DELAY);
		//PORTB = 0x40;
		//_delay_ms(DELAY);
		//insert loop delay for debounce
		//make PORTA an input port with pullups 
		//enable tristate buffer for pushbutton switches
		//now check each button and increment the count as needed
		//bound the count to 0 - 1023
		//break up the disp_value to 4, BCD digits in the array: call (segsum)
		//bound a counter (0-4) to keep track of digit to display 
		//make PORTA an output
		//send 7 segment code to LED segments
		//send PORTB the digit to display
		//update digit to display
	}//while
}//main
