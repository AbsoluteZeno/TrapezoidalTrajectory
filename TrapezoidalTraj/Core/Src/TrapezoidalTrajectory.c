/*
 * TrapezoidalTrajectory.c
 *
 *  Created on: Jun 2, 2023
 *      Author: AbsoluteZeno
 */
#include "TrapezoidalTrajectory.h"
#include "arm_math.h"

extern float q_des;
extern float qdot_des;
extern float qddot_des;

float v_max = 1000.0;	  	  // mm/s
float a = 2000.0;		  	  // mm/s^

void TrapezoidalTraj_PreCal(int16_t start_pos, int16_t final_pos, Traj* trajectory)
{
	if (start_pos != final_pos)
	{
		float s = final_pos - start_pos;

		trajectory->t_acc = v_max/a;
		trajectory->t_total = (pow(v_max,2)+a*fabs(s))/(a*v_max);
	}
}

void TrapezoidalTraj_GetState(int16_t start_pos, int16_t final_pos, Traj* trajectory, uint32_t t_us)
{
	if (start_pos != final_pos)
	{
		float t = t_us/1000000.0;

		float t_acc = trajectory->t_acc;
		float t_total = trajectory->t_total;

		float s = final_pos - start_pos;
		int8_t dir = 1;
		if (s < 0)
		{
			dir = -1;
		}

		if (2*t_acc < t_total) // General Case
		{
			if (t <= t_acc)
			{
				qddot_des = dir*a;
				qdot_des = dir*a*t;
				q_des = start_pos + dir*(0.5*a*pow(t,2));
			}
			else if (t_acc < t && t < (t_total - t_acc))
			{
				qddot_des = 0;
				qdot_des = dir*a*t_acc;
				q_des = start_pos + dir*(0.5*a*pow(t_acc,2) + a*t_acc*(t - t_acc));
			}
			else if ((t_total - t_acc) <= t && t <= t_total)
			{
				qddot_des = -dir*a;
				qdot_des = dir*a*(t_total - t);
				q_des = start_pos + dir*(a*t_total*t+a*t_acc*t_total-a*pow(t_acc,2)-0.5*a*(pow(t,2)+pow(t_total,2)));
			}
		}
		else	// Triangle Case
		{
			t_acc = 0.5*sqrt(4*fabs(s)/a);
			t_total = 2*t_acc;

			if (t <= t_acc)
			{
				qddot_des = dir*a;
				qdot_des = dir*a*t;
				q_des = start_pos + dir*(0.5*a*pow(t,2));
			}
			else if (t_acc < t && t < t_total)
			{
				qddot_des = -dir*a;
				qdot_des = dir*a*(2*t_acc - t);
				q_des = start_pos + dir*(2*a*t_acc*t-0.5*a*pow(t,2)-a*pow(t_acc,2));
			}
		}
	}
}

