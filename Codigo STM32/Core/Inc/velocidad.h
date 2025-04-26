/*
 * velocidad.h
 *
 *  Created on: Apr 7, 2025
 *      Author: Andres
 */

#ifndef INC_VELOCIDAD_H_
#define INC_VELOCIDAD_H_

#include "main.h"
#include <math.h>


void Encoder_1_Init(TIM_HandleTypeDef *tim_encoder1);
void Encoder_2_Init(TIM_HandleTypeDef *tim_encoder2);

void vel(int16_t rpm_s);
int16_t* calculo_rpm();
int32_t conversor(float volt);
int16_t conversor_mm_s(int32_t velocidad);
//void control_distancia(uint16_t distancia_objetivo_mm, uint16_t velocidad_objetivo_mm, uint16_t aceleracion_objetivo_mm);




#endif /* INC_VELOCIDAD_H_ */
