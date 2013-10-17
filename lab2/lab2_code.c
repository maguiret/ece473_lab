// lab2_skel.c 
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define DELAY 2
#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12];


//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
uint8_t chk_buttons(uint8_t button) {

	return 0;
}
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
  //determine how many digits there are 
  //break up decimal sum into 4 digit-segments
  //blank out leading zero digits 
  //now move data to right place for misplaced colon position
}//segment_sum
//***********************************************************************************

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}

//uint8_t switch_process(uint8_t *cnt)
//{
//	uint8_t ret = (0xFF & ~(1 << *cnt));
//	if (*cnt == 8)
//		*cnt = 0;
//	else
//		*cnt++;
//
//	return ret;
//}


//***********************************************************************************
int main()
{
	uint8_t ii;
	uint8_t counter = 0;
	
	//set port bits 4-7 B as outputs
	DDRA = 0xFF;
	DDRB = 0xFF;
	DDRD = 0x00;

	PORTA = 0xFF;
	PORTB = 0x00; //PWM low, 
	PORTD = 0xFF;

	while (!debounce_switch());

	while(1){
		if (debounce_switch()) {
			//counter = (counter < 8 ? counter++ : 0);
			if (counter > 7) {
				counter = 0;
			} else {
				counter++;
			}
		}
		PORTA = 0xFF & ~(1 << counter);

		PORTB = 0x00;
		_delay_ms(DELAY);
		PORTB = 0x10;
		_delay_ms(DELAY);
		PORTB = 0x20;
		_delay_ms(DELAY);
		PORTB = 0x30;
		_delay_ms(DELAY);
		PORTB = 0x40;
		_delay_ms(DELAY);
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
