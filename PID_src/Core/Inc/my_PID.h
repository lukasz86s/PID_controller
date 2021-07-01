/*
 * PID.h
 *
 *  Created on: 1 lip 2021
 *      Author: fet
 */

#ifndef INC_MY_PID_H_
#define INC_MY_PID_H_

#include <stdint.h>

float proportional(float err, float K);
float integral(float err, float time_s, float K);
float derivative(float err, float time_s, float K);
float get_PID(float err, float time_s, float Kp , float Ki, float Kd);


#endif /* INC_MY_PID_H_ */
