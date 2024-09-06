/*
 * location.cpp
 *
 *  Created on: Sep 2, 2024
 *      Author: mac
 */
/*
         front
           90
           y
            ︿
           |
           |
           |
 180--------------->x 0
           |
           |
           |
          -90
 */
#include "location.h"
#include "motor.h"
#include "pathSensor.h"
#include "stdlib.h"
#include "cmath"

#define pi 3.14159

extern float path_dis_x, path_dis_y, path_motor_speed[2];

//length:cm，time:s
float wheel_radius = 10;
float chassis_radius = 31.86625;
float timer_span = 0.001;

//wheel angular speed
float chassis_right_wheel_angspeed;
float chassis_left_wheel_angspeed;

//chassis speed
float chassis_speed;
float chassis_angspeed;

//encoder read
float encRead[2];

//map info
float map_x = 0;
float map_y = 0;
float map_theta_front = pi / 2;
float map_theta_back = 0;
//target info
float goal_x = 0;
float goal_y = 0;
float goal_theta = 0;

//last distance info
float last_x = 0;
float last_y = 0;
float last_theta = 0;

float SP = 0, spin = 0;

int mode = 0;
int check_sf = 0;
int check_sb = 0;
int check_f = 0;
int check_b = 0;
int check_a = 0;
int check_e = 0;

bool arrive = 0;

void cis_speedTransfer_modle(){

	//rps
	encodersp(encRead);

	// 0:right, 1:left
	//cm/s
	chassis_speed 	 = ((encRead[0] + encRead[1]) / 2) * (2 * pi * wheel_radius) / 2;

	//rps
	chassis_angspeed = ((encRead[0] - encRead[1]) / 2) * wheel_radius / chassis_radius / 2;
}
//input cm/s, rps
void trans_speedTransfer_modle(float chassis_sp, float chassis_angsp){

	//rps
	chassis_right_wheel_angspeed = (chassis_sp / (2 * pi) + chassis_radius * chassis_angsp) / wheel_radius;
	chassis_left_wheel_angspeed  = (chassis_sp / (2 * pi) - chassis_radius * chassis_angsp) / wheel_radius;
}
void location_reset(){

	map_x = 0;
	map_y = 0;
	map_theta_front = pi / 2;
	map_theta_back = -1 * pi / 2;

	goal_x = 0;
	goal_y = 0;
	goal_theta = 0;

	last_x = 0;
	last_y = 0;
	last_theta = 0;

	chassis_right_wheel_angspeed = 0;
	chassis_left_wheel_angspeed = 0;

	arrive = 0;
}
//update location info 0 path, 1 integral, 2 stop
void location_data(int MODE){

	cis_speedTransfer_modle();

	//integral location
	if(MODE == 1){
		last_x = goal_x - map_x;
		last_y = goal_y - map_y;
		last_theta = std::fmin(std::abs(goal_theta - map_theta_front), std::abs(goal_theta - map_theta_back));

		map_x += chassis_speed * timer_span * std::cos(map_theta_front);
		map_y += chassis_speed * timer_span * std::sin(map_theta_front);

		//record orientation change(rad)
		map_theta_front += chassis_angspeed * timer_span * 2 * pi;

		if(map_theta_front > 0)
			map_theta_back = map_theta_front - pi;
		else
			map_theta_back = map_theta_front + pi;

		//rad:+pi ~ -pi
		if(map_theta_front >= pi || map_theta_front <= -1 * pi)
			map_theta_front *= -1;
	}
	//path location
	else if(MODE == 0){
		last_x = path_dis_x - map_x;
		last_y = path_dis_y - map_y;

		if(!path_dis_x)
			map_x += chassis_speed * timer_span;
		else
			map_y += chassis_speed * timer_span;
	}
}
//goto (x,y), orientation 180 ~ -180 degree，input 1000 to not spin
void integral_moveto(float x, float y, float orientation){

	bool direction = 1;

	arrive = 0;
	mode = 0;

	SP = 0;
	spin = 0;

	goal_x = x;
	goal_y = y;
	goal_theta = orientation * pi / 180;

	location_data(1);

	//forward or backward
	if(std::abs(map_theta_front - std::atan2(last_y, last_x)) > pi / 2 &&
			std::abs(map_theta_front - std::atan2(last_y, last_x)) < 1.5 * pi)
		direction = 0;

	while(!arrive){

		//spin forward
		if(std::abs(map_theta_front - std::atan2(last_y, last_x)) > pi / 180 && (std::abs(last_x) > 1 ||
				std::abs(last_y) > 1) && direction){

			mode = 1;
			check_sf++;

			if(map_theta_front > std::atan2(last_y, last_x) ||
			(std::abs(map_theta_front - std::atan2(last_y, last_x)) > pi * 1.5 && map_theta_front < 0))
				trans_speedTransfer_modle(0, -1 * spin);
			else
				trans_speedTransfer_modle(0, spin);

			arrive = 0;
		}
		//forward
		else if((std::abs(last_x) > 1 || std::abs(last_y) > 1) && direction){

			mode = 2;
			check_f++;

			trans_speedTransfer_modle(SP, 0);

			arrive = 0;
		}
		//spin backward
		else if(std::abs(map_theta_back - std::atan2(last_y, last_x)) > pi / 180 && (std::abs(last_x) > 1 ||
				std::abs(last_y) > 1) && !direction){

			mode = 1;
			check_sb++;

			if(map_theta_back > std::atan2(last_y, last_x) ||
			(std::abs(map_theta_back - std::atan2(last_y, last_x)) > pi * 1.5 && map_theta_back < 0))
				trans_speedTransfer_modle(0, -1 * spin);
			else
				trans_speedTransfer_modle(0, spin);

			arrive = 0;
		}
		//backward
		else if((std::abs(last_x) > 1 || std::abs(last_y) > 1) && !direction){

			mode = 2;
			check_b++;

			trans_speedTransfer_modle(-1 * SP, 0);

			arrive = 0;
		}
		//spin to specific orientation(front)
		else if(std::abs(map_theta_front - goal_theta) > pi / 180 && orientation != 1000){

			mode = 1;
			check_a++;

			if(std::abs(map_theta_front - goal_theta) < pi){

				if(map_theta_front > goal_theta)
					trans_speedTransfer_modle(0, -1 * spin);
				else
					trans_speedTransfer_modle(0, spin);
			}
			else{
				if(map_theta_front > goal_theta)
					trans_speedTransfer_modle(0, spin);
				else
					trans_speedTransfer_modle(0, -1 * spin);
			}

			arrive = 0;
		}
		//achieve
		else{

			mode = 3;
			check_e++;
			trans_speedTransfer_modle(0, 0);

			arrive = 1;
		}
	}
}
//0 path, 1 integral, 2 stop
void speed_change(int MODE){

	//integral
	if(MODE == 1){
		//speed up
		if(mode == 2 && last_x > 15 && last_y > 15 && SP <= 80)
			SP += 0.02;
		//slow down
		else if(mode == 2 && last_x < 15 && last_y < 15 && SP >= 1)
			SP -= 0.02;
		//angular speed up
		else if(mode == 1 && last_theta > pi / 35 && spin <= 0.3)
			spin += 0.01;
		//angular slow down
		else if(mode == 1 && last_theta < pi / 35 && spin >= 0.01)
			spin -= 0.01;
	}
	//path
	else if(MODE == 0){

		arrive = 0;
		//spin
		if(path_motor_speed[0] * path_motor_speed[1] < 0){

			chassis_right_wheel_angspeed = path_motor_speed[0];
			chassis_left_wheel_angspeed  = path_motor_speed[1];
		}
		//stop
		else if(path_motor_speed[0] * path_motor_speed[1] == 0 || (std::abs(last_x) < 1 && std::abs(last_y) < 1)){

			chassis_right_wheel_angspeed = 0;
			chassis_left_wheel_angspeed  = 0;

			arrive = 1;
		}
		//speed up
		else if((!path_dis_x && last_y > 15) || (!path_dis_y && last_x > 15)){

			if(chassis_right_wheel_angspeed <= path_motor_speed[0])
				chassis_right_wheel_angspeed += 0.001;

			if(chassis_left_wheel_angspeed <= path_motor_speed[1])
				chassis_left_wheel_angspeed += 0.001;
		}
		//slow down
		else if((!path_dis_x && last_y < 15) || (!path_dis_y && last_x < 15)){

			if(chassis_right_wheel_angspeed >= path_motor_speed[0] / 5)
				chassis_right_wheel_angspeed -= 0.001;

			if(chassis_left_wheel_angspeed >= path_motor_speed[1] / 5)
				chassis_left_wheel_angspeed -= 0.001;
		}
	}
	//stop
	else{
		chassis_right_wheel_angspeed = 0;
		chassis_left_wheel_angspeed  = 0;
	}
}
