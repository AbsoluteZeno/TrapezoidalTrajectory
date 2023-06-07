/*
 * joyStick.h
 *
 *  Created on: Jun 6, 2023
 *      Author: msi1
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#include "ModBusRTU.h"
#include "stm32f4xx_hal.h"

uint64_t micros();
void GetJoystickXYaxisValue(uint8_t* ptrx, uint8_t* ptry);
void JoyStickControlCartesian();

#endif /* INC_JOYSTICK_H_ */
