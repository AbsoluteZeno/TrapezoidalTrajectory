/*
 * Controller.c
 *
 *  Created on: Jun 2, 2023
 *      Author: AbsoluteZeno
 */
#include "Controller.h"
#include "Encoder.h"

extern QEIStructureTypeDef QEIData;
extern float q_des;
extern float PulseWidthModulation;

float setposition = 0;
float first_error = 0;
float second_error = 0;
float third_error = 0;

void PositionControlVelocityForm(PID* controller)
{
	first_error = q_des - QEIData.position;

	float kp_position = controller->Kp;
	float ki_position = controller->Ki;
	float kd_position = controller->Kd;

	PulseWidthModulation += ((kp_position + ki_position + kd_position) * first_error) - ((kp_position + (2 * kd_position)) * second_error) + (kd_position * third_error);

	third_error = second_error;
	second_error = first_error;
}

