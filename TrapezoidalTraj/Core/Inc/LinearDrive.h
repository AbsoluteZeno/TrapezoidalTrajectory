/*
 * LinearDrive.h
 *
 *  Created on: Jun 2, 2023
 *      Author: AbsoluteZeno
 */

#ifndef SRC_LINEARDRIVE_H_
#define SRC_LINEARDRIVE_H_
#include "main.h"

void MotorDrive(TIM_HandleTypeDef* PWM_tim);
void SetHome(TIM_HandleTypeDef* Encoder_tim, TIM_HandleTypeDef* PWM_tim);

#endif /* SRC_LINEARDRIVE_H_ */
