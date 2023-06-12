/*
 * enddffector.c
 *
 *  Created on: Jun 7, 2023
 *      Author: msi1
 */

#include "EndEffector.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "BaseSystemStateMachine.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern uint16_t GPIO_Pin;

extern uint8_t action;
extern uint32_t timestamp;
extern uint32_t timestamp2;
extern uint32_t timestamp3;

extern uint8_t tx[40];
extern uint8_t rx[1];

extern uint16_t duty;

extern uint16_t joy_xy[2];
extern uint8_t sw;
extern uint8_t joystick_reading;

extern uint8_t pe1_st;
extern uint8_t pe2_st;
extern uint8_t pe3_st;

extern uint8_t emer_pushed;

extern uint32_t enc_raw;

extern uint8_t softReset_cmd[];
extern uint8_t emerMode_cmd[];
extern uint8_t exitEmer_cmd[];
extern uint8_t testMode_cmd[];
extern uint8_t exitTest_cmd[];
extern uint8_t runMode_cmd[];
extern uint8_t exitRun_cmd[];
extern uint8_t pickup_cmd[];
extern uint8_t place_cmd[];

extern uint8_t		EffAllOff_Flag;
extern uint8_t		EffLaserOn_Flag;
extern uint8_t		EffGripperOn_Flag;
extern uint8_t		EffGripperPick_Flag;
extern uint8_t		EffGripperPlace_Flag;

extern uint16_t 	EffRegState;

extern uint8_t effstatus[1];
extern uint8_t effstatus_temp;
extern uint8_t effreg_temp;

extern u16u8_t registerFrame[70];

void led_fnc()
{
	if(emer_pushed == 1)
	{
		if(rx[0] == '1' && action == 0)
		{
			action = 1;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		}
		if(rx[0] == '2' && action == 0)
		{
			action = 1;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		}
		if(rx[0] == '3' && action == 0)
		{
			action = 1;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
		}
	}
}

void motor_fnc(TIM_HandleTypeDef htim1)
{
	if(emer_pushed == 1)
	{
		if(rx[0] == '5' && action == 0)
		{
			action = 1;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
		}
		if(rx[0] == '7' && action == 0)
		{
			action = 1;
			duty += 50;
		}
		if(rx[0] == '4' && action == 0)
		{
			action = 1;
			duty -= 50;
		}
		if(rx[0] == '9' && action == 0)
		{
			action = 1;
			duty = 0;
		}
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_12 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0){
		emer_pushed = 0;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
		effreg_temp = registerFrame[2].U16;
		effstatus_temp = effstatus[0];
		eff_write(emerMode_cmd);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	}
	if(GPIO_Pin == GPIO_PIN_12 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 1){
		eff_write(exitEmer_cmd);
		registerFrame[2].U16 = effreg_temp;
		effstatus[0] = effstatus_temp;

		EffRegState = registerFrame[2].U16;

		switch(EffRegState){
		case 0b0000:	//everything off
			EffAllOff_Flag = 1;
			break;
		case 0b0001:	//laser on
			EffLaserOn_Flag = 1;
			break;
		case 0b0010:	//gripper on
			EffGripperOn_Flag = 1;
			break;
		case 0b0110:	//gripper picking
			EffGripperPick_Flag = 1;
			break;
		case 0b1010:	//gripper placing
			EffGripperPlace_Flag = 1;
			break;
		}

		BaseSystem_EffAllOff();
		BaseSystem_EffLaserOn();
		BaseSystem_EffGripperOn();
		BaseSystem_EffGripperPick();
		BaseSystem_EffGripperPlace();

		emer_pushed = 1;
	}
}

void check_pe()
{
	if(emer_pushed == 1)
	{
		pe1_st = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
		pe2_st = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
		pe3_st = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	}
}

void check_js()
{
	if(emer_pushed == 1)
	{
		if(joystick_reading == 0)
		{
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)joy_xy, 2);
			joystick_reading = 1;
		}
		sw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	}
}

//void check_emer(){
//	emer_st = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
//    sprintf((char*)tx,"Emergency Switch is at state : %d\r\n",emer_st);
//    HAL_UART_Transmit(&huart2, tx, strlen((char*)tx), 10);
//}

void print_st()
{
	if(timestamp2 < HAL_GetTick())
	{
		timestamp2 = HAL_GetTick() + 200;
		uint8_t led1,led2,led3,dir;
		uint16_t cyc;
		uint8_t st[300];
		led1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		led2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
		led3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
		dir  = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
		cyc  = duty;
		//pe1_st,pe2_st,pe3_st,joy_xy[0],joy_xy[1],sw
		sprintf((char*)st,"\n\n\n\n\n\n\n\n\n\n\n\n\n\rled1 : %d\r\nled2 : %d\r\nled3 : %d\r\nmotor direction : %d\r\nduty cycle : %d\r\nphotoelectric 1 : %d\r\nphotoelectric 2 : %d\r\nphotoelectric 3 : %d\r\njoy x : %d\r\njoy y : %d\r\njoy sw : %d\r\n",led1,led2,led3,dir,cyc,pe1_st,pe2_st,pe3_st,joy_xy[1],joy_xy[0],sw);
		HAL_UART_Transmit(&huart2, st, 185, HAL_MAX_DELAY);
	}
}

void test_eff(){
	if(emer_pushed == 1){
//		extern uint8_t softReset_cmd[];
//		extern uint8_t emerMode_cmd[];
//		extern uint8_t exitEmer_cmd[];
//		extern uint8_t testMode_cmd[];
//		extern uint8_t runMode_cmd[];
//		extern uint8_t exitRun_cmd[];
//		extern uint8_t pickup_cmd[];
//		extern uint8_t place_cmd[];
		if(rx[0] == 'q' && action == 0){
			action = 1;
			eff_write(softReset_cmd);
		}
		else if(rx[0] == 'w' && action == 0){
			action = 1;
			eff_write(emerMode_cmd);
		}
		else if(rx[0] == 'e' && action == 0){
			action = 1;
			eff_write(exitEmer_cmd);
		}
		else if(rx[0] == 'r' && action == 0){
			action = 1;
			eff_write(testMode_cmd);
		}
		else if(rx[0] == 't' && action == 0){
			action = 1;
			eff_write(runMode_cmd);
		}
		else if(rx[0] == 'y' && action == 0){
			action = 1;
			eff_write(exitRun_cmd);
		}
		else if(rx[0] == 'u' && action == 0){
			action = 1;
			eff_write(pickup_cmd);
		}
		else if(rx[0] == 'i' && action == 0){
			action = 1;
			eff_write(place_cmd);
		}

	}
}

void eff_write(uint8_t* cmd2){
	HAL_I2C_Master_Transmit_IT(&hi2c1, 0x15 << 1, cmd2, 4);
}

void eff_write2(uint8_t* cmd3){
	HAL_I2C_Master_Transmit_IT(&hi2c1, 0x15 << 1, cmd3, 8);
}

void eff_read(){
	HAL_I2C_Master_Receive_IT(&hi2c1, 0x15 << 1, effstatus, 1);
}
