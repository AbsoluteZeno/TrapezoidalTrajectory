/*
 * endeffector.h
 *
 *  Created on: Jun 7, 2023
 *      Author: msi1
 */

#ifndef INC_ENDEFFECTOR_H_
#define INC_ENDEFFECTOR_H_

#include "main.h"

void led_fnc();
void motor_fnc();
void check_pe();
void check_js();
void print_st();
void test_eff();
void eff_write(uint8_t* cmd);
void eff_write2(uint8_t* cmd3);
void eff_read();

#endif /* INC_ENDEFFECTOR_H_ */
