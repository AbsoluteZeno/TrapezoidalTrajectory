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
extern uint16_t effst_mb;

void eff_st(){
	//if placing  (bit10==10) => bit3 = 1
	//if picking  (bit10==01) => bit2 = 1
	//if runMode  (bit2==1)   => bit1 = 1
	//if testMode (bit3==1)   => bit0 = 1

	//				READ	REG
	//alloff		0000	0000
	//laser on		1000	0001
	//gripper on	0100	0010
	//pick			0101	0110
	//picked		0111	N/A
	//place			0110	1010

	eff_read();
	effstatus = effstatus & 0b00001111;
	if     (effstatus == 0b0000)	{effst_mb = 0b0000;}
	else if(effstatus == 0b1000)	{effst_mb = 0b0001;}
	else if(effstatus == 0b0100)	{effst_mb = 0b0010;}
	else if(effstatus == 0b0101)	{effst_mb = 0b0110;}
	//else if(effstatus == 0b0111)	{effst_mb = }
	else if(effstatus == 0b0110)	{effst_mb = 0b1010;}

	registerFrame[2].U16 = effst_mb;
}
