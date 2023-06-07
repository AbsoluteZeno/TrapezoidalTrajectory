/*
 * holePositionsCartesian.h
 *
 *  Created on: Jun 6, 2023
 *      Author: msi1
 */

#ifndef INC_HOLEPOSITIONSCARTESIAN_H_
#define INC_HOLEPOSITIONSCARTESIAN_H_
#include "arm_math.h"

void swap(float* a, float* b);
void SetTwoPointsForCalibrate(float x0, float y0, float x1, float y1, uint8_t trayNumber); // 0->Pick, 1->Place
void HolePositionsCartesian(float32_t* bottomleft, float32_t rotationAngleRadian, float32_t* holePositionsCartesian);

#endif /* INC_HOLEPOSITIONSCARTESIAN_H_ */
