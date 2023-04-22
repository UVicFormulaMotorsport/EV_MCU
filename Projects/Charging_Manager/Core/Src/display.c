/*
 * display.c
 *
 *  Created on: Oct. 13, 2022
 *      Author: byo10
 */

/* Hoo lord oh boyy, its time to deal with the display and all the UI functionality :)
 * We should display:
 * Current Battery voltage
 * Current Charging Current
 * Current Charging Power
 * Max voltage
 * Max Current
 * State of charge
 * Hi temp
 * Lo temp
 * I got a list of stuff
 *
 *
 *
 *
 */

#include "main.h"
#include "display.h"

extern I2C_HandleTypeDef hi2c1; //HAL I2C handle

extern int16_t current_current;//values to display
extern int8_t minimum_cell_temp;
extern int8_t max_cell_temp;
extern uint8_t state_of_charge;

char display_contents[41]; //what shows on the display, 40 chars plus null terminator

// A utility function to reverse a string
char *reverse(char *str)
{
  char *p1, *p2;

  if (! str || ! *str)
        return str;
  for (p1 = str, p2 = str + strlen(str) - 1; p2 > p1; ++p1, --p2)
  {
        *p1 ^= *p2; //cursed pointer manipulation
        *p2 ^= *p1;
        *p1 ^= *p2;
  }
  return str;
}

char *ftoa(float f, uint8_t decimalplace, char *buf){ //yeah this is really something. Dont worry about it :) This does not concern you
	if(f<0){
		return NULL;
	}
	int ipart = (int)f;
	itoa(ipart,buf,10);
	if(strlen(buf) > 3){
		if(buf[3]>4){
			buf[2] += 1;
		}
	}

	return buf;

}

//init LCD
void init_LCD(){
//TODO everything
	uint8_t lcd_init_data[lcd_init_len] = {};
	if(HAL_I2C_Master_Transmit(&hi2c1, _LCD_ADDRESS >> 1, lcd_init_data, lcd_init_len, I2C_TIMEOUT_LEN)){

	}


	int i;
	for(i = 0; i<40; i++){
		display_contents[i] = ' '; //init the display to be all spaces, with null terminator at the end
	}
	display_contents[i] = '\0';

	if(generate_string()){
		//oh lawd it messed up
	}

	return; //look for error codes
}

uint8_t generate_string(){
//TODO write code for creating the display string

	return 0;
}

//write to display
