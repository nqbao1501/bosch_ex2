/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm_seca.h"

void generate_seed(){
	for (int i = 0; i < 6; i++){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);

        seed[i] = (uint8_t)((adc_val ^ (TimeStamp << i)) & 0xFF);
        HAL_Delay(1);
	}

	key[0] = seed[0] ^ seed[1];
	key[1] = seed[1] + seed[2];
	key[2] = seed[2] ^ seed[3];
	key[3] = seed[3] + seed[0];
	key[4] = seed[4] & 0xF0;
	key[5] = seed[5] & 0x0F;
}


bool validate_key(uint8_t key_from_user[6]){

	if (memcmp(key_from_user, key, 6) == 0) return true;
	else return false;
}
void SID_27_Practice(){
	uint8_t data_buffer[8];
	if (((CAN1_DATA_RX[0] >> 4) & 0xFF) == 0x00) {

		SID = CAN1_DATA_RX[1];
		uint8_t	sub_func = CAN1_DATA_RX[2];
		uint8_t	len = CAN1_DATA_RX[0];

		if (sub_func == 1){
			if (len != 2){
				//Loi 0x13: sai format
				prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x13);
				return;
			}

			if (security_access_granted){
				//goi seed khi he thong da mo roi -> loi 0x10
				prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x10);
				return;
			}

			seed_sent = true;
			generate_seed();
			data_buffer[0] = SID + 0x40;
			data_buffer[1] = 0x01;
			memcpy(&data_buffer[2],seed, 6);
			prepare_CAN_First_Frame(CAN1_DATA_TX, data_buffer, 8);
			return;
		}
		else if (sub_func ==2){
			prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x13);
			return;
		}
		else{
			prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x10);
			return;
		}
	}

	if (((CAN1_DATA_RX[0] >> 4) & 0xFF) == 0x01) {
			uint16_t len = ((CAN1_DATA_RX[0] & 0x0F) << 8) | CAN1_DATA_RX[1];
			uint8_t SID = CAN1_DATA_RX[2];
			uint8_t sub_func = CAN1_DATA_RX[3];

			if (sub_func == 2){
				if (len != 8){
					//Loi 0x13: sai format
					prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x13);
					seed_sent = !seed_sent;
					return;
				}

				if (!seed_sent){
					//Chua gui 27 01 de lay seed ma da gui 27 02 voi key -> loi 0x10
					prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x10);
					return;
				}

			memcpy(key_from_user, &CAN1_DATA_RX[4], 4);
			prepare_CAN_Flow_Control_Frame(CAN1_DATA_TX);
			}

			else {
				prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x10);
				return;
			}
	}

	if (((CAN1_DATA_RX[0] >> 4) & 0xFF) == 0x02) {
		memcpy(&key_from_user[4], &CAN1_DATA_RX[1], 2);
		if (!validate_key(key_from_user)){
			//key khong dung voi seed -> loi 0x35
			prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x35);
			seed_sent = !seed_sent;
			return;
		}

		//Dung key -> positive response
	    data_buffer[0] = SID + 0x40;
	    data_buffer[1] = 0x02;
	    prepare_CAN_TX_frame(CAN1_DATA_TX, data_buffer, 2);
		security_access_granted = true;
		return;
	}

	if (((CAN1_DATA_RX[0] >> 4) & 0xFF) == 0x03) {
		memcpy(&data_buffer[0], &seed[4], 2);
		prepare_CAN_Consecutive_Frames (CAN1_DATA_TX, data_buffer, 2);
	}

}

