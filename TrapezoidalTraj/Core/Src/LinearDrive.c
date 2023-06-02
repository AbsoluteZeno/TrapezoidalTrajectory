/*
 * LinearDrive.c
 *
 *  Created on: Jun 2, 2023
 *      Author: AbsoluteZeno
 */
#include "arm_math.h"
#include "LinearDrive.h"
#include "Encoder.h"

extern QEIStructureTypeDef QEIData;
extern float PulseWidthModulation;
extern uint8_t emer_pushed;
extern uint8_t 	pe1_st;
extern uint8_t 	pe2_st;					//Photoelectric Sensor Value
extern uint8_t 	pe3_st;
extern uint8_t SetHomeFlag;
extern uint8_t P_disallow;
extern uint8_t N_disallow;

void MotorDrive(TIM_HandleTypeDef* PWM_tim)
{
	if(emer_pushed == 1){
		if (PulseWidthModulation >= 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			N_disallow = 0;
			if (PulseWidthModulation > 8000)
			{
				PulseWidthModulation = 8000;
			}

			if (pe2_st || P_disallow)
			{
				PulseWidthModulation = 0;
				P_disallow = 1;
			}
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			P_disallow = 0;
			if (PulseWidthModulation < -8000)
			{
				PulseWidthModulation = -8000;
			}

			if (pe3_st || P_disallow)
			{
				PulseWidthModulation = 0;
				N_disallow = 1;
			}
		}

		__HAL_TIM_SET_COMPARE(PWM_tim, TIM_CHANNEL_1, fabs(PulseWidthModulation*5));
	}
}

void SetHome(TIM_HandleTypeDef* Encoder_tim, TIM_HandleTypeDef* PWM_tim)
{
	static enum {Jog, Overcenter, Center, Recenter} SetHomeState = Jog;

	if (SetHomeFlag)
	{
		switch (SetHomeState)
		{
		case Jog:
			PulseWidthModulation = 3000;

			if (pe1_st)
			{
				__HAL_TIM_SET_COUNTER(Encoder_tim, 0);
				SetHomeState = Overcenter;
			}
			else if (pe2_st)
			{
				__HAL_TIM_SET_COUNTER(Encoder_tim, 0);
				SetHomeState = Recenter;
			}
			break;
		case Overcenter:
			PulseWidthModulation = 3000;

			if (QEIData.position >= 50)
			{
				SetHomeState = Center;
			}
			break;
		case Center:
			PulseWidthModulation = -3000;

			if (pe1_st)
			{
				PulseWidthModulation = 0;
				MotorDrive(PWM_tim);
				__HAL_TIM_SET_COUNTER(Encoder_tim, 0);
				SetHomeFlag = 0;
				SetHomeState = Jog;
			}
			break;
		case Recenter:
			PulseWidthModulation = -3000;

			if (QEIData.position <= -300)
			{
				SetHomeState = Center;
			}
			break;
		}
		MotorDrive(PWM_tim);
	}
}
