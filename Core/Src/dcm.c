/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm.h"
/*for further services please add service header here*/
#include "dcm_rdbi.h"
#include "dcm_wdbi.h"
#include "dcm_seca.h"

void prepare_CAN_TX_frame(uint8_t* CAN_TX_frame, uint8_t* data, uint8_t data_count){
	CAN_TX_frame[0] = data_count;

	for (uint8_t i = 1; i <= data_count; i++){
		CAN_TX_frame[i] = data[i - 1];
	}
	//Padding những bit còn lại bằng 0x55
	for (uint8_t j = 1; j < (8 - data_count); j++){
		CAN_TX_frame[(data_count + j)] = 0x55;
	}

}

void CAN2_SendMessage(uint8_t* data)
{
    uint32_t txMailbox;
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &txMailbox);
}

void CAN1_SendMessage(uint8_t* data)
{
    uint32_t txMailbox;
	HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, &txMailbox);
}
void prepare_negetive_response_buffer(uint8_t* CAN_TX_frame, uint8_t* buffer, uint8_t SID, uint8_t NRC){
	buffer[0] = 0x7F;
	buffer[1] = SID;
	buffer[2] = NRC;

	prepare_CAN_TX_frame(CAN_TX_frame, buffer, 3);
}

