/*
 * Controller.h
 *
 *  Created on: Jun 3, 2023
 *      Author: AbsoluteZeno
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_
#include "main.h"

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
}PID;

void PositionControlVelocityForm(PID* controller);

#endif /* INC_CONTROLLER_H_ */
