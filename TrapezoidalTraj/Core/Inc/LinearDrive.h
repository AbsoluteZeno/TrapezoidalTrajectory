/*
 * LinearDrive.h
 *
 *  Created on: Jun 3, 2023
 *      Author: AbsoluteZeno
 */

#ifndef INC_LINEARDRIVE_H_
#define INC_LINEARDRIVE_H_
#include "main.h"

void MotorDrive(TIM_HandleTypeDef* PWM_tim);
void SetHome(TIM_HandleTypeDef* Encoder_tim, TIM_HandleTypeDef* PWM_tim);

#endif /* INC_LINEARDRIVE_H_ */
