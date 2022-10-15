#include "driving_loop_task.h"
#include "main.h"
#include <stdio.h>

// PDEAL MAP CONSTANTS
const uint8_t REGEN = 0;
const float GAMMA = 1;
const uint32_t A_PEDAL_RAW_MAX = 4095;
const uint32_t B_PEDAL_RAW_MAX = 4095;
const uint8_t MAX_CONT_TORQUE = 125;
const uint8_t MAX_PEAK_TORQUE = 240;

// VARS FOR PEDAL MAP
float dmd_trq; // Demanded torque
float mtr_spd = 0; // motor speed
float max_cnt_pwr = 100; // max continuous power
float output_percent; // percentage of current max torque to output
uint16_t trq_setpoint; // Torque setpoint
uint16_t StatusFlags;

// CANBUS VARS
CAN_TxHeaderTypeDef setpointTxHeader; //CAN Tx Header
CAN_TxHeaderTypeDef aPedPTxHeader; // Accelerator Pedal % Tx Header
CAN_TxHeaderTypeDef aPedRTxHeader; // Accelerator Pedal Raw Tx Header
CAN_TxHeaderTypeDef bPedPTxHeader; // Brake Pedal % Tx Header
CAN_TxHeaderTypeDef bPedRTxHeader; // Brake Pedal Raw Tx Header
CAN_TxHeaderTypeDef flagsTxHeader; // Implausible state Tx header

// Returns true if the car is not in or recovering from an implausible state
int plausibility_check(float ap_percent, float bp_percent, uint8_t *IMPLAUSIBLE_STATE)
{
	// If in an implausible state, check if the accelerator pedal
	// has dropped below 5 percent, in this case, update state and return true
	// otherwise return false

	// TODO: Check for short or open circuit
	if (*IMPLAUSIBLE_STATE == TRUE)
	{
		if (ap_percent < 5)
		{
			*IMPLAUSIBLE_STATE = FALSE;
			StatusFlags = 0;
			CANsend(&hcan1, &flagsTxHeader, StatusFlags, &pTxMailbox);
			HAL_Delay(10);
			return TRUE;
		}
		else
		{
			return FALSE;
		}
	}
	else
	{
		// Check ap and bp to see if they are currently in an implausible state.
		// Change the status and return false if they are
		// TODO: Figure out the bp_percent value for mechanical brake actuation
		if (ap_percent >= 25 && bp_percent >= 90)
		{
			*IMPLAUSIBLE_STATE = TRUE;
			StatusFlags = 0xFFFF;
			CANsend(&hcan1, &aPedPTxHeader, StatusFlags, &pTxMailbox);
			return FALSE;
		}
		else
		{
			return TRUE;
		}
	}
}

// Calculate the demanded torque based on the pedal sensor actuation
// Based on equation from section 3.3.3 of the paper linked below
// https://www.researchgate.net/publication/323522409_Design_and_realization_of_a_One-Pedal-Driving_algorithm_for_the_TUe_Lupo_EL
float pedal_map(float throttle, float max_trq)
{
	float val = ((throttle - REGEN) / (max_trq - REGEN));
	float trq_dmd = pow(val, GAMMA);
	return trq_dmd;
}

// Returns the maximum continuous torque available at the current rpm
float cont_torque_map(uint16_t rpm)
{
	// ax*2 + bx + c
	float val = (-0.000003 * rpm * rpm) + (0.0192 * rpm) + (97.429);
	if (val < 0)
		return 0;
	if (val > MAX_CONT_TORQUE)
		return MAX_CONT_TORQUE;
	return val;
}

// Returns the maximum peak torque available at the current rpm
float peak_torque_map(uint16_t rpm)
{
	// ax*2 + bx + c
	float val = (-0.000001 * rpm * rpm) + (0.0029 * rpm) + (239.57);
	if (val < 0)
		return 0;
	if (val > MAX_PEAK_TORQUE)
		return MAX_PEAK_TORQUE;
	return val;
}

// Calculate a setpoint value given a torque value
uint16_t calculate_setpoint(float dmd_trq)
{
	float output_percent = dmd_trq / 240;
	return output_percent * (float) 32767;
}


void driving_loop_task(void *arguments)
{
	float ap_raw = 0; // raw accelerator pedal sensor value
	float bp_raw = 0; // raw brake pedal sensor value
	float ap_percent = 0; // accelerator pedal value as a percent
	float bp_percent = 0; // brake pedal value as a percent
	int max_trq = 0; // Maximum available torque from the motor

	//Initialize CAN headers - standard id type, set standard Id = filter ID of other device
	CAN_TxHeader_Init(&setpointTxHeader, 3, CAN_ID_STD, CAN_RTR_DATA, MC_TX);
	CAN_TxHeader_Init(&aPedPTxHeader, 4, CAN_ID_STD, CAN_RTR_DATA, 0x301);
	CAN_TxHeader_Init(&aPedRTxHeader, 4, CAN_ID_STD, CAN_RTR_DATA, 0x302);
	CAN_TxHeader_Init(&bPedPTxHeader, 4, CAN_ID_STD, CAN_RTR_DATA, 0x303);
	CAN_TxHeader_Init(&bPedRTxHeader, 4, CAN_ID_STD, CAN_RTR_DATA, 0x304);
	CAN_TxHeader_Init(&flagsTxHeader, 2, CAN_ID_STD, CAN_RTR_DATA, 0x201);

	// Get ADC values
	HAL_ADC_Start(&hadc1);
	ap_raw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	bp_raw = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	printf("AP_RAW - %f, AP_PERCENT - %f\n\r", ap_raw, ap_percent); //Print to what??

	// Convert [sensor value] -> [% activation]
	ap_percent = (ap_raw / A_PEDAL_RAW_MAX) * 100;
	bp_percent = (bp_raw / B_PEDAL_RAW_MAX) * 100;

	uint32_t ap_percent_i = (uint32_t) ap_percent;
	uint32_t bp_percent_i = (uint32_t) bp_percent;
	uint32_t ap_raw_i = (uint32_t) ap_raw;
	uint32_t bp_raw_i = (uint32_t) bp_raw;

	char msg[100];

	// Send pedal position can messages
	CANsend(&hcan1, &aPedPTxHeader, ap_percent_i, &pTxMailbox); // A pedal percent
	CANsend(&hcan1, &aPedRTxHeader, ap_raw_i, &pTxMailbox); // A pedal raw
	CANsend(&hcan1, &bPedPTxHeader, bp_percent_i, &pTxMailbox); // B pedal percent
	CANsend(&hcan1, &bPedRTxHeader, bp_raw_i, &pTxMailbox); // B pedal raw

	//Check pedal plausibility
	// If in a plausible state, convert pedal sensor actuation to desired torque
	if (plausibility_check(ap_percent, bp_percent, &IMPLAUSIBLE_STATE))
	{

		// Get current max torque
		// TODO: Read motor rpm
		/*
		 if (PEAK_TORQUE_MODE)
		 {
		 max_trq = peak_torque_map(current_rpm);
		 }
		 else
		 {
		 max_trq = cont_torque_map(current_rpm);
		 }
		 */

		max_trq = 100;
		dmd_trq = pedal_map(ap_percent, max_trq);
	}
	else
	{
		// If not in plausible state, set demanded torque to 0
		// TODO: May need to trigger shutdown circuit here, not sure.
		dmd_trq = 0;
	}

	// Calculate the torque setpoint
	trq_setpoint = calculate_setpoint(dmd_trq);

	// Send CAN message
	// Split the setpoint value into 2 byte blocks
	uint32_t trq_setpoint_mod = (trq_setpoint << 8) | 0x90;
	CANsend(&hcan1, &setpointTxHeader, trq_setpoint_mod, &pTxMailbox);

	// Convert to string and print to serial
	sprintf(msg, "AP_RAW - %lu, AP_PERCENT - %lu, BP_RAW - %lu, BP_PERCENT - %lu, SP - %i\r\n", ap_raw_i, ap_percent_i,
			bp_raw_i, bp_percent_i, trq_setpoint);
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 1000);

	HAL_Delay(250);

	return;
}
