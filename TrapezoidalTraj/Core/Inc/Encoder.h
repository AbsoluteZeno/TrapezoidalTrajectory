/*
 * Encoder.h
 *
 *  Created on: Jun 2, 2023
 *      Author: AbsoluteZeno
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_
#include "main.h"

typedef struct _QEIStructure
{
	int16_t pos[2];
	int16_t vel[2];
	uint32_t timestamp[2];
	float position;	// mm
	float velocity;	// mm/s
	float accelaration; // mm/s^2
}QEIStructureTypeDef;

uint64_t micros(TIM_HandleTypeDef* Timer_tim);
void QEIEncoderPositionVelocity_Update(TIM_HandleTypeDef* Encoder_tim, TIM_HandleTypeDef* Timer_tim);

#endif /* INC_ENCODER_H_ */
