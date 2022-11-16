/*
 * display.h
 *
 *  Created on: Oct. 13, 2022
 *      Author: byo10
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#define _LCD_ADDRESS 0x78

typedef struct display_list_node{
	struct display_list_node *next;
//	struct display_list_node *prev;
	char display[41]; // String to display on the screen

}display_list_node;

typedef struct display_list{
	struct display_list_node *head;
	struct display_list_node *tail;
	struct display_list_node *cur;

}displaylist;


//func prototypes
void init_LCD();
void I2C_out(unsigned char);
void I2C_Start(void);
void I2C_Stop(void);
void Show(unsigned char);

#endif /* INC_DISPLAY_H_ */

