/*
 * pathSensor.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
#include "pathSensor.h"
#include "location.h"
#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern float map_x, map_y, last_x, last_y;
extern bool arrive;

#define normal_Speed 1
#define w_kp 0.21
#define w_kd 0
#define boundry 1
#define spin_sp 0.8

uint16_t adcRead[7];
int   check = 0;
float weight_err;
float weight_lasttime = 0;
float weight_change = 0;
float tempSpeed[2];
float path_motor_speed[2];
float path_dis_x = 0, path_dis_y = 0;
/*
adcRead[0]  adc1-0   PA0  right
adcRead[1]  adc1-1   PA1    |
adcRead[2]  adc1-4   PA4    |
adcRead[3]  adc1-6   PA6    V
adcRead[4]  adc1-7   PA7  left
adcRead[5]  adc1-8   PB0  middle right
adcRead[6]  adc1-9   PB1  middle left
*/
void path_setup(){
	if(HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adcRead,7) != HAL_OK)
		check++;
}
//motor_speed[0]:right motor speed, motor_speed[1]:left motor speed
void path(){

	//P's err
	weight_err = (float)(-3*adcRead[0]-adcRead[1]+adcRead[3]+3*adcRead[4])/
					(adcRead[0]+adcRead[1]+adcRead[2]+adcRead[3]+adcRead[4]);
	//D's err
	weight_change = weight_err - weight_lasttime;

	weight_lasttime = weight_err;

	//right
	tempSpeed[0] = normal_Speed + weight_err * w_kp + weight_change * w_kd;
	//left
	tempSpeed[1] = normal_Speed - weight_err * w_kp - weight_change * w_kd;

	//turn right
	if(adcRead[5] >= boundry && adcRead[6] < boundry && adcRead[0] < boundry && adcRead[1] < boundry
			&& adcRead[2] < boundry && adcRead[3] < boundry && adcRead[4] < boundry){

		path_motor_speed[0] = spin_sp * -1;
		path_motor_speed[1] = spin_sp;

		while(adcRead[2] < 3 * boundry){}
	}
	//turn left
	else if(adcRead[5] < boundry && adcRead[6] >= 2 * boundry && adcRead[0] < boundry && adcRead[1] < boundry
			&& adcRead[2] < boundry && adcRead[3] < boundry && adcRead[4] < boundry){

		path_motor_speed[0] = spin_sp;
		path_motor_speed[1] = spin_sp * -1;

		while(adcRead[2] < 3 * boundry){}
	}
	//stop
	else if(adcRead[5] >= boundry && adcRead[6] >= boundry){

		path_motor_speed[0] = 0;
		path_motor_speed[1] = 0;
	}
	//forward
	else{
		for(int j = 0; j < 2; j++){
			for(float i = 2; i >= 0; i -= 0.01){

				if(tempSpeed[j] >= i){
					path_motor_speed[j] = i;
					break;
				}
				else if(tempSpeed[j] > 2){
					path_motor_speed[j] = 2;
					break;
				}
				else if(tempSpeed[j] < 0){
					path_motor_speed[j] = 0;
					break;
				}
			}
		}
	}
}
//go to (x,y)
void path_moveto(float path_x, float path_y){

	path_dis_x = path_x;
	path_dis_y = path_y;

	while(!arrive)
		path();
}
