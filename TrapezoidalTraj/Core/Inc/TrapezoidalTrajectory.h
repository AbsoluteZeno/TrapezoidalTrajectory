/*
 * TrapezoidalTrajectory.h
 *
 *  Created on: Jun 2, 2023
 *      Author: AbsoluteZeno
 */

#ifndef INC_TRAPEZOIDALTRAJECTORY_H_
#define INC_TRAPEZOIDALTRAJECTORY_H_
#include "main.h"

typedef struct
{
	float t_total;			  // [s]
	float t_acc;			  // [s]
}Traj;

void TrapezoidalTraj_PreCal(int16_t start_pos, int16_t final_pos, Traj* trajectory);
void TrapezoidalTraj_GetState(int16_t start_pos, int16_t final_pos, Traj* trajectory, uint32_t t_us);
void QuinticTraj_PreCal(int16_t start_pos, int16_t final_pos, Traj* trajectory);
void QuinticTraj_GetState(int16_t start_pos, int16_t final_pos, Traj* trajectory, uint32_t t_us);

#endif /* INC_TRAPEZOIDALTRAJECTORY_H_ */
