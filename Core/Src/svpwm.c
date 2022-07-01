/*
 * svpwm.c
 *
 *  Created on: May 18, 2022
 *      Author: tammy
 */

#include "svpwm.h"
#include "main.h"

//void execute_svpwm(double angle)
//{
//
//}

/*
void execute_phase(int phase,int speed)
{
	//int speed = 1700;
	switch (phase) {
		case 0:
			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
			TIM3->CCR3 = 0;
			TIM4->CCR1 = 0;
			TIM4->CCR2 = 0;
			TIM4->CCR3 = 0;
			break;
		case 1:
			TIM3->CCR1 = 0;
			TIM3->CCR2 = speed;
			TIM3->CCR3 = speed;
			TIM4->CCR1 = speed;
			TIM4->CCR2 = 0;
			TIM4->CCR3 = 0;
			break;
		case 2:
			TIM3->CCR1 = 0;
			TIM3->CCR2 = 0;
			TIM3->CCR3 = speed;
			TIM4->CCR1 = speed;
			TIM4->CCR2 = speed;
			TIM4->CCR3 = 0;
			break;
		case 3:
			TIM3->CCR1 = speed;
			TIM3->CCR2 = 0;
			TIM3->CCR3 = speed;
			TIM4->CCR1 = 0;
			TIM4->CCR2 = speed;
			TIM4->CCR3 = 0;
			break;
		case 4:
			TIM3->CCR1 = speed;
			TIM3->CCR2 = 0;
			TIM3->CCR3 = 0;
			TIM4->CCR1 = 0;
			TIM4->CCR2 = speed;
			TIM4->CCR3 = speed;
			break;
		case 5:
			TIM3->CCR1 = speed;
			TIM3->CCR2 = speed;
			TIM3->CCR3 = 0;
			TIM4->CCR1 = 0;
			TIM4->CCR2 = 0;
			TIM4->CCR3 = speed;
			break;
		case 6:
			TIM3->CCR1 = 0;
			TIM3->CCR2 = speed;
			TIM3->CCR3 = 0;
			TIM4->CCR1 = speed;
			TIM4->CCR2 = 0;
			TIM4->CCR3 = speed;
			break;
		case 7:
			TIM3->CCR1 = speed;
			TIM3->CCR2 = speed;
			TIM3->CCR3 = speed;
			TIM4->CCR1 = speed;
			TIM4->CCR2 = speed;
			TIM4->CCR3 = speed;
			break;

	}

}
*/
