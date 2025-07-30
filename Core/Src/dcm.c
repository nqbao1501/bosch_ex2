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

void prepare_CAN_First_Frame (uint8_t* CAN_TX_frame, uint8_t* data, uint16_t data_count) {
	if (data_count > 0x0FFF) {
		data[0] -= 0x40;
		prepare_negative_response_buffer(CAN_TX_frame, data, data[0], 0x10);
		return; }

	CAN_TX_frame[0] = (0x01 << 4) | ((data_count >> 8) & 0x0F);
	CAN_TX_frame[1] = data_count & 0xFF;

	for (uint8_t i = 2; i < 8; i++){
			CAN_TX_frame[i] = data[i - 2];
		}
}

void prepare_CAN_Flow_Control_Frame (uint8_t* CAN_TX_frame) {
	if (consecutive_sequence_number > block_size) {
		CAN_TX_frame[0] = (0x03 << 4) | 0x2;
	}
	else CAN_TX_frame[0] = (0x03 << 4) | 0x0;
	CAN_TX_frame[1] = block_size;
	CAN_TX_frame[2] = ST_min;
	for (uint8_t i = 3; i < 8 ; i++){
			CAN_TX_frame[i] = 0x55;
		}
	consecutive_sequence_number = 1;
}

void prepare_One_CAN_Consecutive_Frame (uint8_t* CAN_TX_frame, uint8_t* data, uint8_t data_count) {
	CAN_TX_frame[0] = (0x02 << 4) | (consecutive_sequence_number & 0x0F);
	for (uint8_t i = 1; i <= data_count; i++){
			CAN_TX_frame[i] = data[i - 1];
		}
	for (uint8_t j = 1; j < (8 - data_count); j++){
		CAN_TX_frame[(data_count + j)] = 0x55;
	}
	consecutive_sequence_number++;
}

void prepare_CAN_Consecutive_Frames (uint8_t* CAN_TX_frame, uint8_t* data, uint16_t data_count) {
    uint16_t offset = 0;

    while (offset < data_count) {
        uint8_t chunk_size = (data_count - offset >= 7) ? 7 : (data_count - offset);

        // Prepare one frame
        prepare_One_CAN_Consecutive_Frame(CAN_TX_frame, &data[offset], chunk_size);

        // Send CAN frame here
        if (CAN_TX_frame == CAN1_DATA_TX) {
            CAN1_SendMessage(CAN_TX_frame);
		    USART3_SendString((uint8_t *)"ECU: ");
            PrintCANLog(CAN1_pHeader.StdId, CAN_TX_frame);
        }
        if (CAN_TX_frame == CAN2_DATA_TX) {
			CAN2_SendMessage(CAN_TX_frame);
		    USART3_SendString((uint8_t *)"TESTER: ");
			PrintCANLog(CAN2_pHeader.StdId, CAN_TX_frame);
        }


        offset += chunk_size;

        // Respect STmin delay here if necessary
        HAL_Delay(ST_min);  // If ST_min is in milliseconds
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
void prepare_negative_response_buffer(uint8_t* CAN_TX_frame, uint8_t* buffer, uint8_t SID, uint8_t NRC){
	buffer[0] = 0x7F;
	buffer[1] = SID;
	buffer[2] = NRC;

	prepare_CAN_TX_frame(CAN_TX_frame, buffer, 3);
}

