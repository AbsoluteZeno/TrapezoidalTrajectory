/*
 * Encoder.c
 *
 *  Created on: Jun 2, 2023
 *      Author: AbsoluteZeno
 */
#include "arm_math.h"
#include "Encoder.h"

extern uint64_t _micros;
extern QEIStructureTypeDef QEIData;

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
	QEIData.data[0] = lastposition;
//	if (lastposition > (QEI_PERIOD/2) - 1)
//	{
//		QEIData.data[0] = lastposition - QEI_PERIOD;
//	}

	// position calculation
	QEIData.position = QEIData.data[0] * PI *  pulley_dia/res;

	int32_t diffPosition = QEIData.data[0] - QEIData.data[1];
	float diffTime = QEIData.timestamp[0] - QEIData.timestamp[1];

	// unwrap
	if (diffPosition > QEI_PERIOD>>1) diffPosition -= QEI_PERIOD;
	if (diffPosition < -(QEI_PERIOD>>1)) diffPosition += QEI_PERIOD;

	// velocity calculation
	QEIData.velocity = (diffPosition * 1000000.0 * PI * pulley_dia)/(res * diffTime);

	QEIData.data[1] = QEIData.data[0];
	QEIData.timestamp[1] = QEIData.timestamp[0];
}
