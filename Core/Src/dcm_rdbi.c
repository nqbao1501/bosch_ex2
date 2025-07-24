
/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm_rdbi.h"

void SID_22_Practice(){
	uint8_t SID = CAN1_DATA_RX[1];
	uint8_t	DID_High = CAN1_DATA_RX[2];
	uint8_t	DID_Low = CAN1_DATA_RX[3];
	uint8_t	len = CAN1_DATA_RX[0];
	uint8_t data_buffer[8];

	if (len != 3) {
	    // Error 0x13: Incorrect length
		prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x13);

	    return;
	}

	if (DID_High == 0x01 && DID_Low == 0x23) {
	    data_buffer[0] = SID + 0x40;
	    data_buffer[1] = 0x01;
	    data_buffer[2] = 0x23;
	    data_buffer[3] = (ECU_ID >> 8) & 0xFF;
	    data_buffer[4] = (ECU_ID) & 0xFF;
	    prepare_CAN_TX_frame(CAN1_DATA_TX, data_buffer, 5);
	} else {
	    // Error 0x31: Request out of range
		prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x31);
	}
}

