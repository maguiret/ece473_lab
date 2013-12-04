// lm73_functions.c       

#include <util/twi.h>
#include "lm73_functions.h"
#include <util/delay.h>

#ifndef TRUE
#define TRUE 1
#endif

volatile uint8_t lm73_wr_buf[2];
volatile uint8_t lm73_rd_buf[2];

/*****************************************************************************************
 * Function:		lm73_temp_convert 
 * Description:		Function takes a string, the raw temperature, and a
 * 			 farenheit/celsius decision variable
 * Arguments:		String to hold converted data, raw data, decision boolean
 * Return:		Raw temperature data (arbitrary)
 ****************************************************************************************/
uint8_t lm73_temp_convert(char temp_digits[], uint16_t lm73_temp, uint8_t f_not_c)
{
	uint16_t temp = lm73_temp;

	/* Convert sensor output to celsius */
	temp /= 128;

	if (f_not_c == TRUE) //convert to farenheit if necessary
		temp = (((temp * 9) / 5) + 32);

	itoa((int)temp, temp_digits, 10);

	return temp;

}
