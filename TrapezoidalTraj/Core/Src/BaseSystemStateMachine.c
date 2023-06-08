/*
 * BaseSystemStateMachine.c
 *
 *  Created on: 8 มิ.ย. 2566
 *      Author: msi1
 */
#include "BaseSystemStateMachine.h"
#include "main.h"
#include "Controller.h"
#include "arm_math.h"
#include "joyStick.h"
#include "holePositionsCartesian.h"
#include "EndEffector.h"

extern ModbusHandleTypedef hmodbus;
extern u16u8_t registerFrame[70];
extern uint8_t SetHomeYFlag;
extern float Pf;
extern uint8_t SetPickTrayFlag;
extern uint8_t SetPlaceTrayFlag;
extern uint8_t SetHomeFlag;
extern uint8_t RunTrayFlag;
extern uint8_t RunPointFlag;
extern float Pickreference[2];
extern float Pickopposite[2];
extern float32_t PickrotationAngleRadian;
extern float32_t PickrotationAngleDegree;
extern float32_t PickTray9holes[18];
extern float Placereference[2];
extern float Placeopposite[2];
extern float32_t PlacerotationAngleRadian;
extern float32_t PlacerotationAngleDegree;
extern float32_t PlaceTray9holes[18];
extern uint8_t GoalReadyFlag;
extern uint8_t ControllerFinishedFollowFlag;

extern uint8_t softReset_cmd[];
extern uint8_t emerMode_cmd[];
extern uint8_t exitEmer_cmd[];
extern uint8_t testMode_cmd[];
extern uint8_t exitTest_cmd[];
extern uint8_t runMode_cmd[];
extern uint8_t exitRun_cmd[];
extern uint8_t pickup_cmd[];
extern uint8_t place_cmd[];

extern void ControllerState();

void BaseSystem_SetHome()
{
	static enum {idle, sethome} state = idle;

	if (SetHomeFlag)
	{
		switch(state)
		{
		case idle:
			registerFrame[1].U16 = 0b00000000; //bit 2 set home = 0 //base system status
			registerFrame[16].U16 = 0b00000100; //bit 2 set home = 1 //y-axis moving status
			state = sethome;
		break;
		case sethome:
			//set home x-axis
			registerFrame[65].U16 = 0b00000001;
			//set home y-axis
			SetHomeYFlag = 1;
			registerFrame[16].U16 = 0b00000000;//bit 2 set home = 0 //y-axis moving status

			if(registerFrame[64].U16 == 0b00000000)
			{
				state = idle;
				SetHomeFlag = 0;
			}
		break;
		}
	}
}

void BaseSystem_RunPointMode()
{
	static enum {idle, RunPointMode} state = idle;

	if (RunPointFlag)
	{
		switch(state)
		{
		case idle:
			registerFrame[1].U16 = 0b00000000; //bit 4 run point mode = 0 //base system status
			registerFrame[16].U16 = 0b00100000; //bit 5 go point = 1 //y-axis moving status
			state = RunPointMode;
		break;
		case RunPointMode:
			//set point of XY-axis
			registerFrame[65].U16 = registerFrame[48].U16; //position -1400 to 1400
			registerFrame[66].U16 = 2500; //velocity max 3000
			registerFrame[67].U16 = 1; //acceleration 1 2 3
			Pf = registerFrame[49].U16/10.0;
			ControllerState();

			if(ControllerFinishedFollowFlag && (registerFrame[64].U16 == 0))
			{
				state = idle;
				registerFrame[16].U16 = 0b00000000; //bit 5 go point = 0 //y-axis moving status
				RunPointFlag = 0;
			}
		break;
		}
	}
}

void BaseSystem_SetPickTray()
{
	static enum {Prepare, GetFirstPoint, GetSecondPoint} SetPickTrayState = Prepare;

	static int Pickreference_last[2] = {0, 0};
	static int Pickopposite_last[2] = {0, 0};

	if (SetPickTrayFlag)
	{
		switch(SetPickTrayState)
		{
		case Prepare:
			registerFrame[1].U16 = 0b00000;
			registerFrame[16].U16 = 0b000001;
			SetPickTrayState = GetFirstPoint;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			eff_write(testMode_cmd);
		break;
		case GetFirstPoint:
			GetJoystickXYaxisValue(&Pickreference[0], &Pickreference[1]);
			JoyStickControlCartesian();

			if ((Pickreference_last[0] != Pickreference[0]) || (Pickreference_last[1] != Pickreference[1]))
			{
				SetPickTrayState = GetSecondPoint;
			}
		break;
		case GetSecondPoint:

			GetJoystickXYaxisValue(&Pickopposite[0], &Pickopposite[1]);
			JoyStickControlCartesian();

			if ((Pickopposite_last[0] != Pickopposite[0]) || (Pickopposite_last[1] != Pickopposite[1]))
			{

				SetPickTrayState = Prepare;
				SetTwoPointsForCalibrate(Pickreference[0], Pickreference[1], Pickopposite[0], Pickopposite[1], 0);
				registerFrame[32].U16 = (int)(Pickreference[0]*10);
				registerFrame[33].U16 = (int)(Pickreference[1]*10);
				registerFrame[34].U16 = (int)(PickrotationAngleRadian*100);
				registerFrame[16].U16 = 0b000000;
				SetPickTrayFlag = 0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
				eff_write(exitTest_cmd);

			}
		break;
		}
		Pickreference_last[0] = Pickreference[0];
		Pickreference_last[1] = Pickreference[1];
		Pickopposite_last[0] = Pickopposite[0];
		Pickopposite_last[1] = Pickopposite[1];

		if (SetHomeYFlag == 1)
		{
			memset(Pickreference, 0, sizeof(Pickreference));
			memset(Pickopposite, 0, sizeof(Pickopposite));
			memset(Pickreference_last, 0, sizeof(Pickreference_last));
			memset(Pickopposite_last, 0, sizeof(Pickopposite_last));
			SetPickTrayState = GetFirstPoint;
		}
	}
}

void BaseSystem_SetPlaceTray()
{
	static enum {Prepare, GetFirstPoint, GetSecondPoint} SetPlaceTrayState = Prepare;

	static int Placereference_last[2] = {0, 0};
	static int Placeopposite_last[2] = {0, 0};

	if(SetPlaceTrayFlag)
	{
		switch(SetPlaceTrayState)
		{
		case Prepare:
			registerFrame[1].U16 = 0b00000;
			registerFrame[16].U16 = 0b000010;
			SetPlaceTrayState = GetFirstPoint;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
			eff_write(testMode_cmd);
		break;
		case GetFirstPoint:
			GetJoystickXYaxisValue(&Placereference[0], &Placereference[1]);
			JoyStickControlCartesian();

			if ((Placereference_last[0] != Placereference[0]) || (Placereference_last[1] != Placereference[1]))
			{
				SetPlaceTrayState = GetSecondPoint;
			}
		break;
		case GetSecondPoint:

			GetJoystickXYaxisValue(&Placeopposite[0], &Placeopposite[1]);
			JoyStickControlCartesian();

			if ((Placeopposite_last[0] != Placeopposite[0]) || (Placeopposite_last[1] != Placeopposite[1]))
			{

				SetPlaceTrayState = Prepare;
				SetTwoPointsForCalibrate(Placereference[0], Placereference[1], Placeopposite[0], Placeopposite[1], 1);
				registerFrame[35].U16 = (int)(Placereference[0]*10);
				registerFrame[36].U16 = (int)(Placereference[1]*10);
				registerFrame[37].U16 = (int)(PlacerotationAngleRadian*100);
				registerFrame[16].U16 = 0b000000;
				SetPlaceTrayFlag = 0;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
				eff_write(exitTest_cmd);
			}
		break;
		}
		Placereference_last[0] = Placereference[0];
		Placereference_last[1] = Placereference[1];
		Placeopposite_last[0] = Placeopposite[0];
		Placeopposite_last[1] = Placeopposite[1];

		if (SetHomeYFlag == 1)
		{
			memset(Placereference, 0, sizeof(Placereference));
			memset(Placeopposite, 0, sizeof(Placeopposite));
			memset(Placereference_last, 0, sizeof(Placereference_last));
			memset(Placeopposite_last, 0, sizeof(Placeopposite_last));
			SetPlaceTrayState = GetFirstPoint;
		}
	}
}

void BaseSystem_RuntrayMode()
{
	static enum {HolesCalculate, GoPick, Pick, GoPlace, Place} RunTrayState = GoPick;

	static uint8_t i = 0;
	static uint16_t PickDelay = 0;
	static uint16_t PlaceDelay = 0;

	switch(RunTrayState)
	{
	case HolesCalculate:
		registerFrame[1].U16 = 0b00000;
		GoalReadyFlag = 0;
		HolePositionsCartesian(Pickreference, PickrotationAngleRadian, PickTray9holes);
		HolePositionsCartesian(Placereference, PlacerotationAngleRadian, PlaceTray9holes);
		if (GoalReadyFlag)
		{
			RunTrayState = GoPick;
			eff_write(runMode_cmd);
			i = 0;
		}
	break;
	case GoPick:
		// Run X-Axis to Pick Tray
		registerFrame[65].U16 = (int)(PickTray9holes[2*i]*10); //position -1400 to 1400
		registerFrame[66].U16 = 3000; //velocity max 3000
		registerFrame[67].U16 = 1; //acceleration 1 2 3

		// Run Y-Axis to Pick Tray
		registerFrame[16].U16 = 0b001000;	// Y-Axis Moving status -> GoPick
		Pf = PickTray9holes[2*i + 1];
		ControllerState();

		if(ControllerFinishedFollowFlag && (registerFrame[64].U16 == 0))
		{
			RunTrayState = Pick;
			PickDelay = 0;
		}
	break;
	case Pick:
		eff_write(pickup_cmd);
		PickDelay++;

		if (PickDelay >= 1000)
		{
			RunTrayState = GoPlace;
		}
	break;
	case GoPlace:
		// Run X-Axis to Pick Tray
		registerFrame[65].U16 = (int)(PlaceTray9holes[2*i]*10); //position -1400 to 1400
		registerFrame[66].U16 = 3000; //velocity max 3000
		registerFrame[67].U16 = 1; //acceleration 1 2 3

		// Run Y-Axis to Pick Tray
		registerFrame[16].U16 = 0b010000;	// Y-Axis Moving status -> GoPlace
		Pf = PlaceTray9holes[2*i + 1];
		ControllerState();

		if(ControllerFinishedFollowFlag && (registerFrame[64].U16 == 0))
		{
			RunTrayState = Place;
			PlaceDelay = 0;
			i++;
		}
	break;
	case Place:
		eff_write(place_cmd);
		PlaceDelay++;

		if (PickDelay >= 1000)
		{
			RunTrayState = GoPick;
			if(i >= 9)
			{
				RunTrayState = HolesCalculate;
				eff_write(exitRun_cmd);
				RunTrayFlag = 0;
			}
		}
	break;
	}
}









