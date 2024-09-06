/*
 * mainpp.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
#include "mainpp.h"
#include "motor.h"
//#include "communication.h"
#include "location.h"
#include "servo.h"
#include "string.h"
#include "stm32f4xx_hal.h"

#define epsilon 0.1

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart1;

int move_mode = 2;

void setup(){
	//UART_setup();
	HAL_TIM_Base_Start_IT(&htim7);
	DCmotor_setup();
}
void main_function(){
	//arm_run();
	//arm_script();
//    pwm2();
	setup();
//	pwm1();
//	pwm4();
while(1){
//	move_mode = 1;
//	integral_moveto(100, 100, 1000);
}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*htim){

	if (htim -> Instance == TIM7){

		speedOutput(move_mode);
		DCmotor_run();

		//transmit_test();


		//HAL_UART_Transmit_DMA(&huart1, buffer_TX, sizeof(buffer_TX));
		//HAL_UART_Transmit_IT(&huart1, buffer_TX, sizeof(buffer_TX));
	}
}
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart){
	if(huart -> Instance == USART3){
	   receiveSpeed();
	   transmit_test();
	   r++;
	   // HAL_UART_Receive_DMA(&huart1, buffer_RX, sizeof(buffer_RX));
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart -> Instance == USART3){
		t++;
	}
}
*/



