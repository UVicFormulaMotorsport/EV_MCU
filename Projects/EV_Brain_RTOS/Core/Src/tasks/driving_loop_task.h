#include "../program.h"

#define FALSE 0
#define TRUE !(FALSE)

float cont_torque_map(uint16_t);
float peak_torque_map(uint16_t);
uint16_t calculate_setpoint(float);
int plausibility_check(float, float, uint8_t *);
float pedal_map(float, float);
void driving_loop_task(void *arguments);

// CAN functions
extern void CAN_TxHeader_Init(CAN_TxHeaderTypeDef *pTxHeader, uint32_t dlc, uint32_t ide, uint32_t rtr, uint32_t stdId);
extern void CAN_Filter_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef *sFilterConfig, uint32_t fifo, uint32_t highId, uint32_t lowId, uint32_t highMask, uint32_t lowMask, uint32_t scale);
extern HAL_StatusTypeDef CANsend(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, uint32_t, uint32_t *);

// STATE VARS
extern uint8_t IMPLAUSIBLE_STATE;
extern uint8_t PEAK_TORQUE_MODE;
extern uint8_t READY_TO_DRIVE;

// CANBUS VARS
extern CAN_RxHeaderTypeDef pRxHeader; //CAN Rx Header
extern uint32_t pTxMailbox; // Message mailbox
extern CAN_FilterTypeDef sFilterConfig; //CAN filter configuration
extern uint32_t MC_RX;
extern uint32_t MC_TX;

// STM32 peripheral handles
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;
