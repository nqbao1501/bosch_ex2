/*********************************************************/
/*********BOSCH BEA PROGRAM SKELETON DEMO CODE************/
/*********************************************************/

#ifndef _DCM_SECA_H
#define _DCM_SECA_H

#include "dcm.h"
#include <string.h>

extern unsigned int TimeStamp;
extern uint8_t seed[6];
extern uint8_t key[6];
extern uint8_t key_from_user[6];
extern uint8_t SID;
void SID_27_Practice();
bool validate_key(uint8_t key_from_user[6]);
void generate_seed();
#endif
