/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#include "dcm_seca.h"

void generate_seed(){
	for (int i = 0; i < 4; i++){
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint32_t adc_val = HAL_ADC_GetValue(&hadc1);

        seed[i] = (uint8_t)((adc_val) & 0xFF);
        HAL_Delay(1);
	}

	key[0] = seed[0] ^ seed[1];
	key[1] = seed[1] + seed[2];
	key[2] = seed[2] ^ seed[3];
	key[3] = seed[3] + seed[0];
}


bool validate_key(uint8_t key_from_user[4]){

	if (memcmp(key_from_user, key, 4) == 0) return true;
	else return false;
}
void SID_27_Practice(){
	uint8_t SID = CAN1_DATA_RX[1];
	uint8_t	sub_func = CAN1_DATA_RX[2];
	uint8_t	len = CAN1_DATA_RX[0];
	uint8_t data_buffer[8];

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
	    memcpy(&data_buffer[2],seed, 4);
	    prepare_CAN_TX_frame(CAN1_DATA_TX, data_buffer, 6);

		return;
	}

	if (sub_func == 2){
		if (len != 6){
			//Loi 0x13: sai format
			prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x13);
			return;
		}

		if (!seed_sent){
			//Chua gui 27 01 de lay seed ma da gui 27 02 voi key -> loi 0x10
			prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x10);
			return;
		}

		uint8_t key_from_user[4];
		memcpy(key_from_user, &CAN1_DATA_RX[3], 4);

		if (!validate_key(key_from_user)){
			//key khong dung voi seed -> loi 0x35
			prepare_negative_response_buffer(CAN1_DATA_TX, data_buffer, SID, 0x35);
			return;
		}

		//Dung key -> positive response
	    data_buffer[0] = SID + 0x40;
	    data_buffer[1] = 0x02;
	    prepare_CAN_TX_frame(CAN1_DATA_TX, data_buffer, 2);
		security_access_granted = true;
		return;
	}
}

