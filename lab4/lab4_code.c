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
 * - Come back to this
 *****************************************************************************************
 *
 ****************************LAB 4 SPECIFICS**********************************************
 * - Pushbutton interface:
 *   --------------------------------------------------
 *   |  __    __    __    __    __    __    __    __  |
 *   | /  \  /  \  /  \  /  \  /  \  /  \  /  \  /  \ |
 *   | \__/  \__/  \__/  \__/  \__/  \__/  \__/  \__/ |
 *   |   7     6     5     4     3     2     1     0  |
 *   --------------------------------------------------
 *     None:
 *     0: 12/24 hr mode switch (extended feature)
 *     1: Set clock time
 *     2: Set alarm time
 *     3: Alarm on/off
 *     4:
 *     5:
 *     6:
 *     7: Press for snooze
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
//#define COUNT_MAX 1023
#define SECONDS_MAX 86400

#define COLON_AM 0xFC
#define COLON_PM 0xF8

#define TRUE 1
#define FALSE 0

/* 2 cycle delay */
#define DELAY_CLK do{asm("nop");asm("nop");}while(0)

/* Global variable to hold current displayed number */
volatile uint16_t number;

/* Global variable to hold number of seconds elapsed in the day */
volatile uint32_t seconds = 0;

/* Counters for various ISRs */
volatile uint8_t INT0_count = 0;

/* Holds the on/off state of the clock colon */
volatile uint8_t colon_state = 0;

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
	0b00100000, //center colon
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
	ASSR |= (1 << AS0); //enable asynchronous mode (oscillator acts as clock)
	TIMSK |= (1<<TOIE0); //enable timer/counter0 overflow interrupt
	TCCR0 |= (1<<CS00); //normal mode, prescale by 8
}

/*****************************************************************************************
 * Function:		TCNT2_init
 * Description:		Initializes timer/counter 2
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
void TCNT2_init() 
{
	TIMSK |= (1 << OCIE2); //enable timer/counter2 compare interrupt
	TCCR2 |= ((1 << WGM21) | (1 << WGM20) | //set fast PWM mode
		  (1 << COM21) | (1 << COM20) | //inverted PWM (for active low)
		  (1<<CS00)); //prescale by 8
	OCR2 = 0x7F; //Initialize at 50% duty cycle
}

/*****************************************************************************************
 * Function:		TCNT3_init
 * Description:		Initializes timer/counter 3
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
void TCNT3_init() 
{
	ETIMSK |= (1 << OCIE3A); //enable timer/counter3a compare interrupt
	TCCR3A |= ((1 << WGM30) | //set fast PWM 8 bit mode
		   (1 << COM3A1)); //non-inverted PWM (for active high)
	TCCR3B |= ((1 << WGM32) | //set fast PWM 8 bit mode
		   (1 << CS32)); //256 clock prescaler
	OCR3AL = 0x7F; //Initialize at 50% duty cycle, only low is used for 8 bit PWM
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
 * Function:		led_display_init
 * Description:		Initialized upper half of DDRB for brightness pwm and digit select
 * 			 output, initializes DDRA for LED display output
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
void led_display_init()
{
	DDRA = 0xFF;
	DDRB = 0xF0;
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
	if (button == 0) { //am/pm
		pushbutton_mode ^= 0x01; //toggle first bit
		//twelve_hr ^= 0x01; //toggle 12 hour mode
	} else if (button == 1) { //clock set
	 	pushbutton_mode  ^= 0x02; //toggle second bit
	} else if (button == 2) { //alarm set
	 	pushbutton_mode  ^= 0x04; //toggle third bit
	} else if (button == 3) { //alarm armed
		pushbutton_mode ^= 0x08; //toggle fourth bit
	}
}

/*****************************************************************************************
 * Function:		display_digits
 * Description:		Function takes global seconds variable, breaks it up into minutes
 * 			 and hours, displays those. Blinks colon at the seconds interval
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
void display_digits() 
{
	uint32_t tmp_sec = seconds; //tmp variable to modify number for display
	uint8_t cur_value; //current digit value to display
	uint8_t cur_digit = 0; //current digit to display on
	uint8_t colon;

	/* Set Register A */
	PORTA = 0xFF; //pullups
	DDRA = 0xFF; //output
	DELAY_CLK;

	/* Converts seconds to minutes, gets minutes for any given hour with mod */
	uint8_t minutes = ((tmp_sec / 60) % 60);

	/* Converts seconds to hours, will not exceed 24 due to other program logic,
	 * handles both 24 and 12 hour time */
	uint8_t hours = (tmp_sec / 3600);
	
	/* If am/pm mode */
	if ((pushbutton_mode & 0x01) == FALSE) {
		/* Set colon appropriately, convert to 12 hr time */
		if (hours == 0) {
			hours += 12;
			colon = COLON_AM;
		} else if (hours > 0 && hours < 12) {
			colon = COLON_AM;
		} else if (hours == 12) {
			colon = COLON_PM;
		} else {
			hours -= 12;
			colon = COLON_PM;
		}
	} else {
		colon = COLON_AM;
	}

	/* Isolates most and least significant bits of each variable */
	uint8_t min_l = minutes % 10;
	uint8_t min_h = minutes / 10;
	uint8_t hrs_l = hours % 10;
	uint8_t hrs_h = hours / 10;

	/* Loop displays each base 10 digit one by one. Mods by 10 to get digit, displays
	 * encoded digit to 7-seg, divides by 10 to get next digit. Loops until cur_value
	 * is less than 1. */
	for (cur_digit = 0; cur_digit < 5; cur_digit++) {

		PORTA = 0xFF; //clear PORTA
		DELAY_CLK;
		PORTB &= 0x8F; //clear decoder bits
		DELAY_CLK;
		PORTB |= decoder_select[cur_digit]; //set portb decoder bits

		/* Select which value will be displayed on digit */
		if (cur_digit == 0) 
			cur_value = min_l;
		else if (cur_digit == 1) 
			cur_value = min_h;
		else if (cur_digit == 2) 
			cur_value = hrs_l;
		else if (cur_digit == 3) 
			cur_value = hrs_h;

		PORTA = sev_seg_digits[cur_value]; //display digit

		/* Display colon (or not) */
		if (cur_digit == 4 && colon_state == 1)
			PORTA = colon; //colon on
		else if (cur_digit == 4 && colon_state != 1)
			PORTA = 0xFF; //colon off

		_delay_loop_1(200); //delay for about arg*3 cycles
	}
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
//	uint8_t old_PORTA = PORTA;
//	uint8_t old_PORTB = PORTB;
//	uint8_t old_DDRA = DDRA;
//
//	read_buttons();
//
//	PORTA = old_PORTA;
//	PORTB = old_PORTB;
//	DDRA = old_DDRA;
//	DELAY_CLK;
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
//	PORTB = old_PORTB;
//	DELAY_CLK;
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
	/* Save register states */
	uint8_t old_PORTA = PORTA;
	uint8_t old_PORTB = PORTB;
	uint8_t old_DDRA = DDRA;

	static uint8_t pwm_dir = 1;

	INT0_count++;
	if (INT0_count == 128) {
		seconds++;
		if (seconds == SECONDS_MAX)
			seconds = 0;
		colon_state ^= 0x01; //toggle state of colon
		INT0_count = 0;
	}

	if (pwm_dir == 1)
		OCR2++;
	else
		OCR2--;

	if (OCR2 == 0)
		pwm_dir = 1;
	else if (OCR2 == 255)
		pwm_dir = 0;

	read_buttons();

	/* Restore register states */
	PORTA = old_PORTA;
	PORTB = old_PORTB;
	DDRA = old_DDRA;
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
 * Description:		Timer/Counter 2 controls the display dimming PWM. The PWM is
 * 			 controlled purely by the setup, and as such no ISR is required.
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
ISR(TIMER2_COMP_vect) {}

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
	led_display_init(); //initialize led display
	TCNT0_init(); //initialize timer/counter 0
	TCNT2_init(); //initialize timer/counter 2
	TCNT3_init(); //initialize timer/counter 3
	SPI_init(); //initialize SPI master on PORTB 1-3

	/* enable interrupts */
	sei();

	while (1) {
		display_digits();
	}
}
