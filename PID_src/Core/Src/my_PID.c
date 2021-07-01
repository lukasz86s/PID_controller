/*
 * PID.c
 *
 *  Created on: 1 lip 2021
 *      Author: fet
 */

#include "my_PID.h"

float proportional(float err, float K){
	return (K*err);
}

float integral(float err, float time_s, float K){
	static float sum;
	sum += (err * time_s);
	return K*sum;

}
float derivative(float err, float time_s, float K){
	return K*(err/time_s);
}

float get_PID(float err, float time_s, float Kp , float Ki, float Kd){
	return (proportional(err, Kp) + integral(err, time_s, Ki) + derivative(err, time_s, Kd));
}
