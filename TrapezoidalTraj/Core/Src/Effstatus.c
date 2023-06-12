/*
 * effstatus.c
 *
 *  Created on: 8 มิ.ย. 2566
 *      Author: putte
 */

#include "EndEffector.h"
#include "Effstatus.h"
#include "main.h"

extern u16u8_t registerFrame[70];
extern uint8_t effstatus;

void eff_st(){
	//if placing  (bit10==10) => bit3 = 1
	//if picking  (bit10==01) => bit2 = 1
	//if runMode  (bit2==1)   => bit1 = 1
	//if testMode (bit3==1)   => bit0 = 1

	effstatus = eff_read();
	uint16_t effst_mb = 0;
	//check placing
	if((effstatus & 0b00000011) == 0b00000010){effst_mb = effst_mb | 0b0000000000001000;}
	else									  {effst_mb = effst_mb & 0b1111111111110111;}
	//check picking
	if((effstatus & 0b00000011) == 0b00000001){effst_mb = effst_mb | 0b0000000000000100;}
	else									  {effst_mb = effst_mb & 0b1111111111111011;}
	//check runmode
	if((effstatus & 0b00000100) == 0b00000100){effst_mb = effst_mb | 0b0000000000000010;}
	if((effstatus & 0b00000100) == 0b00000000){effst_mb = effst_mb | 0b1111111111111101;}
	//check testmode
	if((effstatus & 0b00001000) == 0b00001000){effst_mb = effst_mb | 0b0000000000000001;}
	if((effstatus & 0b00001000) == 0b00000000){effst_mb = effst_mb | 0b1111111111111110;}

	registerFrame[2].U16 = effst_mb; //Ya 22881
}
