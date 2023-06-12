/*
 * BaseSystemStateMachine.h
 *
 *  Created on: 8 มิ.ย. 2566
 *      Author: msi1
 */

#ifndef BASESYSTEMSTATEMACHINE_H_
#define BASESYSTEMSTATEMACHINE_H_

#include "ModBusRTU.h"

void BaseSystem_SetHome();
void BaseSystem_RunPointMode();
void BaseSystem_SetPickTray();
void BaseSystem_SetPlaceTray();
void BaseSystem_RuntrayMode();

void BaseSystem_EffAllOff();
void BaseSystem_EffLaserOn();
void BaseSystem_EffGripperOn();
void BaseSystem_EffGripperPick();
void BaseSystem_EffGripperPlace();

#endif /* BASESYSTEMSTATEMACHINE_H_ */
