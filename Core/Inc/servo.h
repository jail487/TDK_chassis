/*
 * servo.h
 *
 *  Created on: 2024年8月19日
 *      Author: 88698
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_
#include "stm32f4xx_hal.h"
#include "mainpp.h"
typedef struct {
    float pos;
    float goalAngle;
    float lastAngle;
    int responseTime;
    bool move;
    TIM_HandleTypeDef *htim;
    uint32_t TIM_CHANNEL;

} servo;
void servo_move(servo*servo,float goalAngle,int responseTime);
void servo_run(servo*servo ,int updateFreq);
void blockState(int state_a,int state_b);
void servo_setup();


#endif /* INC_SERVO_H_ */
