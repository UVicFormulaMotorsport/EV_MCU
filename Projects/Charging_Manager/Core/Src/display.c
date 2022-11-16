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

//init LCD
init_LCD(){
//TODO everything
	uint16_t lcd_init_len = 8;
	uint8_t lcd_init_data[lcd_init_len] = {};
	HAL_I2C_Master_Transmit(hi2c1, _LCD_ADDRESS >> 1, lcd_init_data, lcd_init_len);
}

//write to display
