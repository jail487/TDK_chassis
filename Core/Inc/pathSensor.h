/*
 * pathSensor.h
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
#ifndef INC_PATHSENSOR_H_
#define INC_PATHSENSOR_H_

#include "stm32f4xx_hal.h"

void path_setup();
//go to (x,y)
void path_moveto(float path_x, float path_y);

#endif /* INC_PATHSENSOR_H_ */



