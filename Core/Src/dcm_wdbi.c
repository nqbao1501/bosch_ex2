
/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm_wdbi.h"


void SID_2E_Practice(){
	uint8_t len = CAN1_DATA_RX[0];
	uint8_t SID = CAN1_DATA_RX[1];
	uint8_t DID_High = CAN1_DATA_RX[2];
	uint8_t	DID_Low = CAN1_DATA_RX[3];
	uint8_t data_buffer[8];

	uint16_t temp_ECU_ID = ((CAN1_DATA_RX[4] << 8) | CAN1_DATA_RX[5]);

	if (!security_access_granted){
		prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x33);
		return;
	}

	if (len != 5) {
		prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x13);
		return;
	}

	if (DID_High != 0x01 || DID_Low != 0x23) {
		prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x31);
		return;
	}

	if (temp_ECU_ID > 0x07FF) {
		prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x10);
		return;
	}
	ECU_ID = temp_ECU_ID;
	data_buffer[0] = SID + 0x40;
	prepare_CAN_TX_frame(CAN1_DATA_TX, data_buffer, 1);

}
