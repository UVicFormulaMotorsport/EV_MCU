#include "ready_to_drive_task.h"
#include "main.h"
#include <stdio.h>



void ready_to_drive_task (void *arguments)
{
	// TODO: Figure out what signals we need to check for
	// For now it will just be a button press

	// Check if button is pressed
	if ((GPIOA->IDR & GPIO_PIN_0) == TRUE)
	{
		// Send the RTD signal
		GPIOB->ODR |= GPIO_PIN_1;

		// Wait for response
		while (((GPIOB->IDR & GPIO_PIN_0) == FALSE));

		// Response received when sound is finished playing
		// Set signal back to low
		GPIOB->ODR &= ~GPIO_PIN_1;

		READY_TO_DRIVE = TRUE;
	}
	return;
}
