/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#ifndef _DCM_H
#define _DCM_H

#include "main.h"
#include "stm32f4xx_it.h"

#include <stdint.h>
#include <stdbool.h>


extern uint8_t CAN1_DATA_TX[8];
extern uint8_t CAN1_DATA_RX[8];
extern uint8_t CAN2_DATA_TX[8];
extern uint8_t CAN2_DATA_RX[8];

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_TxHeaderTypeDef CAN1_pHeader;
extern CAN_TxHeaderTypeDef CAN2_pHeader;
extern bool security_access_granted ;
extern bool seed_sent ;
extern uint16_t ECU_ID;

extern ADC_HandleTypeDef hadc1;

void prepare_CAN_TX_frame(uint8_t* CAN_TX_frame, uint8_t* data, uint8_t data_count);
void prepare_CAN_First_Frame (uint8_t* CAN_TX_frame, uint8_t* data, uint16_t data_count) ;
void prepare_CAN_Flow_Control_Frame (uint8_t* CAN_TX_frame);
void prepare_One_CAN_Consecutive_Frame (uint8_t* CAN_TX_frame, uint8_t* data, uint8_t data_count);
void prepare_CAN_Consecutive_Frames (uint8_t* CAN_TX_frame, uint8_t* data, uint16_t data_count);
void CAN2_SendMessage(uint8_t* data);
void CAN1_SendMessage(uint8_t* data);
void prepare_negative_response_buffer(uint8_t* CAN_TX_frame, uint8_t* buffer, uint8_t SID, uint8_t NRC);

#endif
