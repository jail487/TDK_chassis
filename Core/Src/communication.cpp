/*
 * communication.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
#include "communication.h"
#include "stdlib.h"
#include "string.h"
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern float sp_a,sp_b,sp_c,sp_d ;
extern float speed_a,speed_b;
int k ,transmit ,receive;
#define epsilon 0.1

uint8_t buffer_TX[2 * sizeof(float)];
uint8_t buffer_RX[2 * sizeof(float)];

HAL_StatusTypeDef UART_Transmit_Two_Floats_DMA(UART_HandleTypeDef *huart, float value1, float value2) {
   // uint8_t buffer[2 * sizeof(float)];
    memcpy(buffer_TX, &value1, sizeof(float));
    memcpy(buffer_TX + sizeof(float), &value2, sizeof(float));
    return HAL_UART_Transmit_DMA(huart, buffer_TX, sizeof(buffer_TX));
}
HAL_StatusTypeDef UART_Transmit_Two_Floats_IT(UART_HandleTypeDef *huart, float value1, float value2) {
   // uint8_t buffer[2 * sizeof(float)];
    memcpy(buffer_TX, &value1, sizeof(float));
    memcpy(buffer_TX + sizeof(float), &value2, sizeof(float));
    return HAL_UART_Transmit_IT(huart, buffer_TX, sizeof(buffer_TX));
}
void UART_setup(){
//	 HAL_UART_Receive_DMA(&huart3, buffer_RX, sizeof(buffer_RX));
	 HAL_UART_Receive_IT(&huart3, buffer_RX, sizeof(buffer_RX));
	// HAL_TIM_Base_Start_IT(&htim6);
}
void receiveSpeed(){
	memcpy(&sp_a, buffer_RX, sizeof(float));
    memcpy(&sp_b, buffer_RX+ sizeof(float), sizeof(float));//rx
//    HAL_UART_Receive_DMA(&huart3, buffer_RX, sizeof(buffer_RX));
    HAL_UART_Receive_IT(&huart3, buffer_RX, sizeof(buffer_RX));
    receive++;
}
void transmit_test(){
//	UART_Transmit_Two_Floats_DMA(&huart3,speed_a,speed_b);
	UART_Transmit_Two_Floats_IT(&huart3,speed_a,speed_b);
		transmit++;
		if (abs(sp_a - 1) <= epsilon && abs(sp_b - 1) <= epsilon){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
		}else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
			}
}



