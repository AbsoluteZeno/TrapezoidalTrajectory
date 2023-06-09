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

float v_max = 9000.0;	  	  // mm/s
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

void QuinticTraj_PreCal(int16_t start_pos, int16_t final_pos, Traj* trajectory)
{
	if (start_pos != final_pos)
	{
		float s = final_pos - start_pos;

		trajectory->t_acc = 0;
		float t_total_v = (15.0*fabs(s))/v_max;
		float t_total_a = 0.5*sqrt((40*sqrt(3)*fabs(s))/(3*a));

		if(t_total_v > t_total_a)
		{
			trajectory->t_total = t_total_v;
		}
		else
		{
			trajectory->t_total = t_total_a;
		}
	}
}

void QuinticTraj_GetState(int16_t start_pos, int16_t final_pos, Traj* trajectory, uint32_t t_us)
{
	if (start_pos != final_pos)
	{
		float t = t_us/1000000.0;

		float t_total = trajectory->t_total;
		float s = final_pos - start_pos;

		float C5 = 6*s/pow(t_total,5);
		float C4 = -15*s/pow(t_total,4);
		float C3 = 10*s/pow(t_total,3);
		float C0 = start_pos;

		q_des = C5*pow(t,5) + C4*pow(t,4) + C3*pow(t,3) + C0;
		qdot_des = 5*C5*pow(t,4) + 4*C4*pow(t,3) + 3*C3*pow(t,2);
		qddot_des = 20*C5*pow(t,3) + 12*C4*pow(t,2) + 6*C3*t;
	}
}
