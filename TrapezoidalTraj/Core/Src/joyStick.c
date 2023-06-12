/*
 * joyStick.c
 *
 *  Created on: Jun 6, 2023
 *      Author: msi1
 */

#include "joyStick.h"
#include "LinearDrive.h"
#include "Encoder.h"

extern ModbusHandleTypedef hmodbus;
extern u16u8_t registerFrame[70];
extern uint16_t adcRawData[20];
extern uint32_t IN0[10]; //Y
extern uint32_t IN1[10]; //X
extern uint8_t JoyStickSwitch;
extern uint32_t X_axis, Y_axis;
extern uint32_t joystickXaxis, joystickYaxis;
extern QEIStructureTypeDef QEIData;
extern uint8_t SetHomeYFlag;
extern float PulseWidthModulation;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;

uint8_t JoyStickSwitch_last = 1;
uint64_t StartTime = 0;

void GetJoystickXYaxisValue(float* ptrx, float* ptry)
{
	JoyStickSwitch = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

	static uint8_t i = 0;
	if(i % 2 == 0)
	{
		IN1[i/2] = adcRawData[i];
		Y_axis += IN1[i/2];
		if(i == 18)
		{
			joystickYaxis = Y_axis/10;
			Y_axis = 0;
		}
	}
	else if(i % 2 == 1)
	{
		IN0[(i-1)/2] = adcRawData[i];
		X_axis += IN0[(i-1)/2];
		if(i == 19)
		{
			joystickXaxis = X_axis/10;
			X_axis = 0;
			i = 0;
		}
	}
	i = (i+1)%20;

	if ((JoyStickSwitch == 0) && (SetHomeYFlag == 0) && (joystickYaxis < 2500) && (joystickYaxis > 1400)&& (joystickXaxis < 2500) && (joystickXaxis > 1400))
	{
		if (JoyStickSwitch_last == 1 && JoyStickSwitch == 0)
		{
			StartTime = micros(&htim5);
			// Keep encoder position xy
			*ptrx = registerFrame[68].U16/10.0; //encoderx
			*ptry = QEIData.position; //encodery25
		}
		else if (JoyStickSwitch_last == 0 && JoyStickSwitch == 0)
		{
			if ((micros(&htim5) - StartTime) >= 2000000)
			{
				//set home x-axis
				registerFrame[64].U16 = 1;
				//set home y-axis
				SetHomeYFlag = 1;
				StartTime = micros(&htim5);
			}
		}
	}
	JoyStickSwitch_last = JoyStickSwitch;
}

void JoyStickControlCartesian()
{
	if (SetHomeYFlag == 0)
	{
		//X-axis
		if(joystickYaxis > 2500) //Left
		{registerFrame[64].U16 = 4;}

		else if(joystickYaxis < 1400) //Right
		{registerFrame[64].U16 = 8;}

		else{registerFrame[64].U16 = 0;}

		//Y-axis
		if(joystickXaxis < 1400) //Front
		{PulseWidthModulation = 3000;}

		else if(joystickXaxis > 2500) //Back
		{PulseWidthModulation = -3000;}

		else{PulseWidthModulation = 0;}

		MotorDrive(&htim1);
	}
}
