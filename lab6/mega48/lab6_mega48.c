/*
 * Created by: Jacob Branaugh
 * Created on: 12/03/2013 18:17 
 *
 * Code for ECE 473 Lab mega48 chip.
 * 
 ****************************HARDWARE SETUP***********************************************
 *
 *****************************************************************************************
 */

//#define F_CPU 8000000 // cpu speed in hertz 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions.h"

#define TRUE 1
#define FALSE 0

/* Defines for farenheit and celcius */
#define F 1
#define C 0

/* Variables for temp sensor stuff */
volatile char uart_temp_string[3];
volatile uint16_t lm73_temp;
volatile extern uint8_t lm73_rd_buf[2];
volatile extern uint8_t lm73_wr_buf[2];


/*****************************************************************************************
 * Function:		lm73_init
 * Description:		Initializes local temp sensor over twi
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
void lm73_init()
{
	lm73_wr_buf[0] = LM73_PTR_TEMP;

	twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);
}

/*****************************************************************************************
 * Function:		read_lm73
 * Description:		Function reads temperature data from sensor over twi and converts
 * 			 farenhiet or celsius based on 3rd argument to lm73_temp_convert
 * Arguments:		None
 * Return:		None
 ****************************************************************************************/
void read_lm73()
{
	twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2);

	lm73_temp = lm73_rd_buf[0];

	lm73_temp <<= 8;
	lm73_temp |= lm73_rd_buf[1];

	lm73_temp_convert(uart_temp_string, lm73_temp, C);
}

/*****************************************************************************************
 ********************************* MAIN FUNCTION *****************************************
 ****************************************************************************************/
int main()
{
	uint8_t cnt;
	uint8_t strcnt = 0;
	uint8_t dump;
	uint8_t read_new_temp = TRUE;

	init_twi();
	lm73_init();
	uart_init();

	sei();

	while (1) {
		if (read_new_temp == TRUE) {
			read_lm73();
			read_new_temp = FALSE;
		}

		dump = uart_getc();

		/* When something comes in through UART, send out temperature data */
		if (dump != 0) {
			while (!(UCSR0A & (1 << UDRE0)));
			UDR0 = uart_temp_string[0];
			while (!(UCSR0A & (1 << UDRE0)));
			UDR0 = uart_temp_string[1];
			read_new_temp = TRUE;
		}
	}
}
