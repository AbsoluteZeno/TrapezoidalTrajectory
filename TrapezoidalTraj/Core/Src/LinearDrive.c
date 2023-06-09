/*
 * LinearDrive.c
 *
 *  Created on: Jun 2, 2023
 *      Author: AbsoluteZeno
 */
#include "arm_math.h"
#include "LinearDrive.h"
#include "Encoder.h"
#include "Controller.h"
#include "ModBusRTU.h"

extern QEIStructureTypeDef QEIData;
extern float PulseWidthModulation;
extern uint8_t emer_pushed;
extern uint8_t 	pe1_st;
extern uint8_t 	pe2_st;					//Photoelectric Sensor Value
extern uint8_t 	pe3_st;
extern uint8_t SetHomeYFlag;
extern uint8_t CenterFlag;
extern uint8_t P_disallow;
extern uint8_t N_disallow;
extern uint8_t ControllerFinishedFollowFlag;
extern PID Controller;
extern float q_des;
extern u16u8_t registerFrame[70];

float Pcenter = 0;
float Ncenter = 0;
float Temp_pos = 0;

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

			if ((pe3_st && (SetHomeYFlag == 0)) || P_disallow)
			{
				__HAL_TIM_SET_COMPARE(PWM_tim,TIM_CHANNEL_1,0);
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

			if ((pe2_st && (SetHomeYFlag == 0)) || N_disallow)
			{
				__HAL_TIM_SET_COMPARE(PWM_tim,TIM_CHANNEL_1,0);
				N_disallow = 1;
			}
		}

		__HAL_TIM_SET_COMPARE(PWM_tim, TIM_CHANNEL_1, fabs(PulseWidthModulation*5));
	}
}

void SetHome(TIM_HandleTypeDef* Encoder_tim, TIM_HandleTypeDef* PWM_tim)
{
	static enum {Jog, Overcenter, PCenter, UnderCenter, NCenter, Center,  Recenter, Setcenter} SetHomeState = Jog;

	if (SetHomeYFlag)
	{
		switch (SetHomeState)
		{
		case Jog:
			Pcenter = 0;
			Ncenter = 0;
			Temp_pos = 0;
			PulseWidthModulation = 3000;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

			if (pe1_st)
			{
				__HAL_TIM_SET_COUNTER(Encoder_tim, 0);
				SetHomeState = Overcenter;
			}
			else if (pe3_st)
			{
				__HAL_TIM_SET_COUNTER(Encoder_tim, 0);
				SetHomeState = Recenter;
			}
			break;
		case Overcenter:
			PulseWidthModulation = 3000;

			if (QEIData.position >= 30)
			{
				SetHomeState = PCenter;
			}
			break;
		case PCenter:
			PulseWidthModulation = -2500;

			if (pe1_st)
			{
				Pcenter = QEIData.position;
				Temp_pos = Pcenter - 30;
				SetHomeState = UnderCenter;
			}
			break;
		case UnderCenter:
			PulseWidthModulation = -3000;

			if (QEIData.position <= Temp_pos)
			{
				SetHomeState = NCenter;
			}
			break;
		case NCenter:
			PulseWidthModulation = 2500;

			if (pe1_st)
			{
				Ncenter = QEIData.position;
				SetHomeState = Center;
			}
			break;
		case Center:
			q_des = ((Pcenter - Ncenter)/2.0) + Ncenter;
			PositionControlVelocityForm(&Controller);

			if (0.03 > fabs(q_des - QEIData.position))
			{
				PulseWidthModulation = 0;
				MotorDrive(PWM_tim);
				SetHomeState = Setcenter;
			}
			break;
		case Setcenter:
			__HAL_TIM_SET_COUNTER(Encoder_tim, 0);
			SetHomeYFlag = 0;
			SetHomeState = Jog;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

			break;
		case Recenter:
			PulseWidthModulation = -3000;

			if (QEIData.position <= -320)
			{
				SetHomeState = PCenter;
			}
			break;
		}
		MotorDrive(PWM_tim);
	}


}
