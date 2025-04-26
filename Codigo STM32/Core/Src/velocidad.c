/*
 * velocidad.c
 *
 *  Created on: Apr 7, 2025
 *      Author: Andres
 */


#include "velocidad.h"
#include "complemento.h"
#include <math.h>
#include "motorDriver.h"

TIM_HandleTypeDef *tim_encoder_1_Global;
TIM_HandleTypeDef *tim_encoder_2_Global;

uint8_t set = 1;

float pulsos;
float pulsos_2;

float pulsos_ant = 0;
float pulsos_act = 0;
float pulsos_ant_2 = 0;
float pulsos_act_2 = 0;
int32_t contador_actual = 0;
int32_t contador_actual_2 = 0;

int16_t rpm = 0;
int16_t rpm_2 = 0;
int16_t rpm_f = 0;


int32_t val_pwm = 0;
int16_t pwm_mm = 0;



float diametro_rueda = 38.2;
int16_t velocidad_mm_s = 0;
int16_t velocidad_rad_s = 0;

int16_t velocidades[3] = {0};





void Encoder_1_Init(TIM_HandleTypeDef *tim_encoder1){
	tim_encoder_1_Global = tim_encoder1;
	HAL_TIM_Encoder_Start(tim_encoder_1_Global, TIM_CHANNEL_ALL);
}

void Encoder_2_Init(TIM_HandleTypeDef *tim_encoder2){
	tim_encoder_2_Global = tim_encoder2;
	HAL_TIM_Encoder_Start(tim_encoder_2_Global, TIM_CHANNEL_ALL);
}


int16_t* calculo_rpm() {

    	contador_actual = (int32_t)tim_encoder_1_Global->Instance->CNT;
    	contador_actual_2 = (int32_t)tim_encoder_2_Global->Instance->CNT;

        //contador_actual = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        //contador_actual_2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);

        pulsos_act = fabsf((float)contador_actual) / 1431.0f;
        pulsos_act_2 = fabsf((float)contador_actual_2) / 1431.0f;
        pulsos = pulsos_act - pulsos_ant;
        pulsos_2 = pulsos_act_2 - pulsos_ant_2;
        pulsos_ant = pulsos_act;
        pulsos_ant_2 = pulsos_act_2;

        float temp_rpm = pulsos * 600.0f;
		rpm = (int16_t)fabsf(temp_rpm);
		rpm_2 = (int16_t)fabsf(pulsos_2 * 600.0f);

        if (rpm < 0) rpm = 0;
        rpm_f = aplicarFiltroFIR(rpm);
        velocidades[0] = rpm_f;
        vel(rpm_f);


    return velocidades;
}

void vel(int16_t rpm_s) {
    velocidad_mm_s = (3.1416 * diametro_rueda * rpm_s) / 60.0;
    velocidad_rad_s = (2 * 3.1416 * rpm_s) / 60.0;

    velocidades[1] = velocidad_mm_s;
    velocidades[2] = velocidad_rad_s;
}

int32_t conversor(float volt) {
    if (volt < 0) {
        volt = 0;
    } else if (volt > 7.5) {
        volt = 7.5;
    } else {
        val_pwm = (int32_t)((volt * (-100)) / (7.5));
    }

    return val_pwm;
}

int16_t conversor_mm_s(int32_t velocidad) {
    if (velocidad > 702) {
        velocidad = 702;
    }
    pwm_mm = ((velocidad * 100) / 702) * (-1);

    return pwm_mm;
}



/*
void control_distancia(uint16_t distancia_objetivo_mm, uint16_t velocidad_objetivo_mm, uint16_t aceleracion_objetivo_mm) {

	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);

    const float perimetro_rueda_mm = 3.1416 * 38.2f;

    float revoluciones = (float)distancia_objetivo_mm / perimetro_rueda_mm;
    int32_t pulsos_objetivo = (int32_t)(revoluciones * 1431);

    tim_encoder_1_Global->Instance->CNT = 0;
    tim_encoder_2_Global->Instance->CNT = 0;


    int8_t pwm = conversor_mm_s(velocidad_objetivo_mm);

    motores(pwm, pwm);

    int32_t pulsos_izq = 0;
    int32_t pulsos_der = 0;

    while ((pulsos_izq < pulsos_objetivo) || (pulsos_der < pulsos_objetivo)) {

    	pulsos_izq = (int32_t)tim_encoder_1_Global->Instance->CNT;
        pulsos_der = (int32_t)tim_encoder_2_Global->Instance->CNT;
    }

    motores(0, 0);

    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
}
*/



/*
void control_distancia(uint16_t distancia_objetivo_mm, uint16_t velocidad_objetivo_mm, uint16_t aceleracion_objetivo_mm) {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);

    const float perimetro_rueda_mm = 3.1416f * 38.2f;
    const float pulsos_por_mm = 1431.0f / perimetro_rueda_mm;
    const float delta_t_s = 0.01f;
    const uint32_t delta_t_us = 10000;

    int32_t pulsos_objetivo = (int32_t)(distancia_objetivo_mm * pulsos_por_mm);

    tim_encoder_1_Global->Instance->CNT = 0;
    tim_encoder_2_Global->Instance->CNT = 0;

    float v_max = (float)velocidad_objetivo_mm;
    float a = (float)aceleracion_objetivo_mm;
    float t_acc = v_max / a;
    float d_acc = 0.5f * a * t_acc * t_acc;
    float t_dec = t_acc;
    float d_dec = d_acc;
    float d_total = (float)distancia_objetivo_mm;

    float t_const = 0.0f;
    float d_const = 0.0f;

    if (d_acc + d_dec <= d_total) {
        d_const = d_total - (d_acc + d_dec);
        t_const = d_const / v_max;
    } else {

        v_max = sqrtf(a * d_total);
        t_acc = v_max / a;
        t_dec = t_acc;
        d_acc = d_total / 2.0f;
        d_dec = d_acc;
        t_const = 0.0f;
        d_const = 0.0f;
    }

    float t_total = t_acc + t_const + t_dec;

    int32_t pulsos_izq = 0;
    int32_t pulsos_der = 0;
    float t_actual = 0.0f;
    float v_actual = 0.0f;
    int16_t pwm_actual = 0;

    uint32_t tiempo_inicio = HAL_GetTick();

    while (t_actual < t_total && (pulsos_izq < pulsos_objetivo || pulsos_der < pulsos_objetivo)) {
        pulsos_izq = (int32_t)tim_encoder_1_Global->Instance->CNT;
        pulsos_der = (int32_t)tim_encoder_2_Global->Instance->CNT;

        if (t_actual < t_acc) {
            v_actual = a * t_actual;
        } else if (t_actual < t_acc + t_const) {
            v_actual = v_max;
        } else {
            float t_dec_actual = t_actual - (t_acc + t_const);
            v_actual = v_max - a * t_dec_actual;
        }

        pwm_actual = conversor_mm_s((int32_t)v_actual);

        motores(pwm_actual, pwm_actual);

        t_actual += delta_t_s;

        while ((HAL_GetTick() - tiempo_inicio) < (uint32_t)(t_actual * 1000.0f)) {
        }
    }

    motores(0, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
}
*/









