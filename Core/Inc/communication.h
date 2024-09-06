/*
 * communication.h
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "stm32f4xx_hal.h"
#include "mainpp.h"
void UART_setup();
HAL_StatusTypeDef UART_Transmit_Two_Floats_DMA(UART_HandleTypeDef *huart, float value1, float value2);
HAL_StatusTypeDef UART_Transmit_Two_Floats_It(UART_HandleTypeDef *huart, float value1, float value2);
void receiveSpeed();
void transmit_test();



#endif /* INC_COMMUNICATION_H_ */
