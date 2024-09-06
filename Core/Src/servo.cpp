/*
 * servo.cpp
 *
 *  Created on: 2024年8月19日
 *      Author: 88698
 */
#include "servo.h"
extern TIM_HandleTypeDef htim9;
int goalAngle;
int responseTime = 1000;
int servoAngle[2] = {50,160};
//F front, B back, R right, M middle, L left
servo servos[2] = {
    {0, 0, 0, 2000, true, &htim9, TIM_CHANNEL_1}, // FR/PA4
    {0, 0, 0, 2000, true, &htim9, TIM_CHANNEL_2}, // BR/PA6

//    {0, 0, 0, 2000, true, &htim9, TIM_CHANNEL_1}, // FL/PA2
//    {0, 0, 0, 2000, true, &htim9, TIM_CHANNEL_2}  // BL/PA3
};

//servo servo[0]{0,0,0,2000,true, &htim3, TIM_CHANNEL_1 };
void servo_setup(){
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}
void servo_move(servo*servo,float goalAngle,int responseTime){
	servo -> goalAngle = goalAngle;
	servo -> responseTime = responseTime;
	servo -> move = true;
}
void servo_run(servo*servo ,int updateFreq){//updateFreq = timer interrupt frequency Hz
	if (servo -> move == true){
		if ((int)servo -> pos == (int)servo -> goalAngle){
    	servo -> move = false;
    	servo -> lastAngle = servo -> goalAngle;
        }else{
         float distance = servo -> goalAngle - servo -> lastAngle;
         servo -> pos += distance/(servo -> responseTime * updateFreq / 1000);
         __HAL_TIM_SET_COMPARE(servo -> htim, servo -> TIM_CHANNEL,600+10*(int)servo -> pos);
         //t++;
        }
	}
}
void blockState(int state_a,int state_b){
	servo_run(&servos[0], 1000);
	servo_run(&servos[1], 1000);
	//true open ,false close
	if (state_a == true){
		servo_move(&servos[0], servoAngle[0],1000);
	}else{
		servo_move(&servos[0], servoAngle[1],1000);
	}
	if (state_b == true){
		servo_move(&servos[1], servoAngle[1],1000);
	}else{
		servo_move(&servos[1], servoAngle[0],1000);
		}
}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim->Instance == TIM6){
//	servo_move(&servo[0],goalAngle,responseTime);
//	servo_run(&servo[0] ,1000);
//	//t++;
//	}
//}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {

    }
}

       // step_init(targetPos, initAngle, targetAngle, targetPosInitialized);




