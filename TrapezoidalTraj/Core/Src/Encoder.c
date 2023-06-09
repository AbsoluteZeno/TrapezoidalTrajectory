/*
 * Encoder.c
 *
 *  Created on: Jun 2, 2023
 *      Author: AbsoluteZeno
 */
#include "arm_math.h"
#include "Encoder.h"
#include "ModBusRTU.h"

extern uint64_t _micros;
extern QEIStructureTypeDef QEIData;
extern u16u8_t registerFrame[70];

uint16_t res = 8192;      	  // Resolution [pulse/revolution]
float pulley_dia = 30.558;	  // mm

uint64_t micros(TIM_HandleTypeDef* Timer_tim)
{
	return __HAL_TIM_GET_COUNTER(Timer_tim)*0.01 + _micros;
}

void QEIEncoderPositionVelocity_Update(TIM_HandleTypeDef* Encoder_tim, TIM_HandleTypeDef* Timer_tim)
{
	QEIData.timestamp[0] = micros(Timer_tim);
	uint32_t lastposition = __HAL_TIM_GET_COUNTER(Encoder_tim);
	QEIData.pos[0] = lastposition;
	QEIData.pos[0] = -QEIData.pos[0];
//	if (lastposition > (QEI_PERIOD/2))
//	{
//		QEIData.pos[0] = lastposition - QEI_PERIOD - 1;
//	}

	// position calculation
	QEIData.position = QEIData.pos[0] * PI *  pulley_dia/res;

	int32_t diffPosition = QEIData.pos[0] - QEIData.pos[1];
	float diffTime = QEIData.timestamp[0] - QEIData.timestamp[1];

	// unwrap
	if (diffPosition > QEI_PERIOD>>1) diffPosition -= QEI_PERIOD;
	if (diffPosition < -(QEI_PERIOD>>1)) diffPosition += QEI_PERIOD;

	// velocity calculation
	QEIData.velocity = (diffPosition * 1000000.0 * PI * pulley_dia)/(res * diffTime);
	QEIData.vel[0] = QEIData.velocity;
	QEIData.accelaration = (QEIData.vel[0] - QEIData.vel[1])/diffTime;

	QEIData.pos[1] = QEIData.pos[0];
	QEIData.vel[1] = QEIData.vel[0];
	QEIData.timestamp[1] = QEIData.timestamp[0];

	registerFrame[17].U16 = (int)(QEIData.position*10);
	registerFrame[18].U16 = (int)(QEIData.velocity*10);
	registerFrame[19].U16 = (int)(QEIData.accelaration*10);
}
