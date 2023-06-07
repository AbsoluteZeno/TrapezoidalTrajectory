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
extern int Pickreference[2];
extern int Pickopposite[2];
extern float32_t PickrotationAngleRadian;
extern float32_t PickrotationAngleDegree;
extern float32_t PickTray9holes[18];
extern int Placereference[2];
extern int Placeopposite[2];
extern float32_t PlacerotationAngleRadian;
extern float32_t PlacerotationAngleDegree;
extern float32_t PlaceTray9holes[18];

float32_t holePositionsCartesianrotation[18];
int reference[2] = {0, 0};
int opposite[2] = {0, 0};
float32_t rotationAngleRadian = 0;
float32_t Degrees = 0;
static float32_t holePositionsRelativetoBottomLeft[18] =
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

void SetTwoPointsForCalibrate(float x0, float y0, float x1, float y1, uint8_t trayNumber) // 0->Pick, 1->Place
{
	if(y0 > y1){swap(&x0, &x1); swap(&y0, &y1);}
	else if(y0 == y1)
	{
		if (x0 > x1){swap(&x0, &x1); swap(&y0, &y1);}
	}

	reference[0] = x0; reference[1] = y0;
	opposite[0] = x1; opposite[1] = y1;

	rotationAngleRadian = atan2(y1 - y0, x1 - x0) - atan2(50, 60);
	Degrees = rotationAngleRadian * (180 / M_PI);

	if (trayNumber == 0)
	{
		Pickreference[0] = reference[0];
		Pickreference[1] = reference[1];
		Pickopposite[0] = opposite[0];
		Pickopposite[1] = opposite[1];
		PickrotationAngleRadian = rotationAngleRadian;
		PickrotationAngleDegree = Degrees;
	}
	else if (trayNumber == 1)
	{
		Placereference[0] = reference[0];
		Placereference[1] = reference[1];
		Placeopposite[0] = opposite[0];
		Placeopposite[1] = opposite[1];
		PlacerotationAngleRadian = rotationAngleRadian;
		PlacerotationAngleDegree = Degrees;
	}
}

void HolePositionsCartesian(float32_t* bottomleft, float32_t rotationAngleRadian, float32_t* holePositionsCartesian)
{
	if (GoalReadyFlag == 0)
	{
		float32_t rotationMatrix[4] =
		{
			arm_cos_f32(rotationAngleRadian),  //0
			arm_sin_f32(rotationAngleRadian),  //1
			-arm_sin_f32(rotationAngleRadian), //2
			arm_cos_f32(rotationAngleRadian)   //3
		};

		static uint8_t i = 0;
		//rotation
		holePositionsCartesianrotation[i*2] = (holePositionsRelativetoBottomLeft[i*2] * rotationMatrix[0]) + (holePositionsRelativetoBottomLeft[i*2+1] * rotationMatrix[2]);
		holePositionsCartesianrotation[i*2+1] = (holePositionsRelativetoBottomLeft[i*2] * rotationMatrix[1]) + (holePositionsRelativetoBottomLeft[i*2+1] * rotationMatrix[3]);

		//translation
		*(holePositionsCartesian + (i*2)) = holePositionsCartesianrotation[i*2] + bottomleft[0];
		*(holePositionsCartesian + (i*2 + 1)) = holePositionsCartesianrotation[i*2+1] + bottomleft[1];

		i++;
		if (i == 9)
		{
			GoalReadyFlag = 1;
			i = 0;
		}
	}
}
