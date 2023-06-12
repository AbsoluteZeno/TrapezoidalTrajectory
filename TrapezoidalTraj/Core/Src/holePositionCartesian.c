/*
 * holePositionCartesian.c
 *
 *  Created on: Jun 6, 2023
 *      Author: msi1
 */
#include "holePositionsCartesian.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

extern uint8_t GoalReadyFlag;
extern float32_t Pickreference[2];
extern float32_t Pickopposite[2];
extern float32_t PickrotationAngleRadian;
extern float32_t PickrotationAngleDegree;
extern float32_t PickTray9holes[18];
extern float32_t Placereference[2];
extern float32_t Placeopposite[2];
extern float32_t PlacerotationAngleRadian;
extern float32_t PlacerotationAngleDegree;
extern float32_t PlaceTray9holes[18];

float32_t holePositionsCartesianrotation[18];
float32_t rotationAngleRadian = 0;
float32_t Degrees = 0;
float holePositionsRelativetoBottomLeft[18] =
{
  10, 10,
  30, 10,
  50, 10,
  10, 25,
  30, 25,
  50, 25,
  10, 40,
  30, 40,
  50, 40
};

void swap(float* a, float* b)
{
	float temp = *a;
	*a = *b;
	*b = temp;
}

void SetTwoPointsForCalibrate(float* x0, float* y0, float* x1, float* y1, uint8_t trayNumber) // 0->Pick, 1->Place
{
	if(*y0 > *y1){swap(x0, x1); swap(y0, y1);}
	else if(*y0 == *y1)
	{
		if (*x0 > *x1){swap(x0, x1); swap(y0, y1);}
	}

	rotationAngleRadian = atan2(50, 60) - atan2(*y1 - *y0, *x1 - *x0);
	Degrees = rotationAngleRadian * (180 / M_PI);

	if (trayNumber == 0)
	{
		PickrotationAngleRadian = rotationAngleRadian;
		PickrotationAngleDegree = Degrees;
	}
	else if (trayNumber == 1)
	{
		PlacerotationAngleRadian = rotationAngleRadian;
		PlacerotationAngleDegree = Degrees;
	}
}

void HolePositionsCartesian()
{
	if (GoalReadyFlag == 0)
	{
		float PickrotationMatrix[4] =
		{
			arm_cos_f32(PickrotationAngleRadian),  //0
			arm_sin_f32(PickrotationAngleRadian),  //1
			-arm_sin_f32(PickrotationAngleRadian), //2
			arm_cos_f32(PickrotationAngleRadian)   //3
		};

		float PlacerotationMatrix[4] =
		{
			arm_cos_f32(PlacerotationAngleRadian),  //0
			arm_sin_f32(PlacerotationAngleRadian),  //1
			-arm_sin_f32(PlacerotationAngleRadian), //2
			arm_cos_f32(PlacerotationAngleRadian)   //3
		};

		static uint8_t i = 0;

		PickTray9holes[i*2] = (holePositionsRelativetoBottomLeft[i*2] * PickrotationMatrix[0]) + (holePositionsRelativetoBottomLeft[i*2+1] * PickrotationMatrix[2]) + Pickreference[0];
		PickTray9holes[i*2 + 1] = (holePositionsRelativetoBottomLeft[i*2] * PickrotationMatrix[1]) + (holePositionsRelativetoBottomLeft[i*2+1] * PickrotationMatrix[3]) + Pickreference[1];

		PlaceTray9holes[i*2] = (holePositionsRelativetoBottomLeft[i*2] * PlacerotationMatrix[0]) + (holePositionsRelativetoBottomLeft[i*2+1] * PlacerotationMatrix[2]) + Placereference[0];
		PlaceTray9holes[i*2 + 1] = (holePositionsRelativetoBottomLeft[i*2] * PlacerotationMatrix[1]) + (holePositionsRelativetoBottomLeft[i*2+1] * PlacerotationMatrix[3]) + Placereference[1];

		i++;
		if (i == 9)
		{
			GoalReadyFlag = 1;
			i = 0;
		}
	}
}
