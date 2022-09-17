#include "../program.h"

#define FALSE 0
#define TRUE !(FALSE)

void ready_to_drive_loop(void *);

// STATE VARS
extern uint8_t IMPLAUSIBLE_STATE;
extern uint8_t PEAK_TORQUE_MODE;
extern uint8_t READY_TO_DRIVE;

// STM32 peripheral handles
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;
