/*
 * distancia.h
 *
 *  Created on: Apr 7, 2025
 *      Author: Andres
 */

#ifndef INC_DISTANCIA_H_
#define INC_DISTANCIA_H_

#include "main.h"
#include <math.h>


#define PI 3.14159265359f
#define DIAMETRO_RUEDA_MM 33.0f
#define RADIO_RUEDA_MM (DIAMETRO_RUEDA_MM / 2.0f)
#define DISTANCIA_ENTRE_RUEDAS_MM 75.0f
#define PULSOS_POR_REVOLUCION 1431



typedef struct {
    float x;               // Posición en X (mm)
    float y;               // Posición en Y (mm)
    float theta;           // Orientación (rad)
    float velMotorL;       // Velocidad motor izquierdo (mm/s)
    float velMotorR;       // Velocidad motor derecho (mm/s)
    float velLineal;       // Velocidad lineal del robot (mm/s)
    float velAngular;      // Velocidad angular del robot (rad/s)
    float avanceLineal;    // Avance acumulado en línea recta (mm)
    float left_distance_mm; // Distancia recorrida por rueda izquierda (mm)
    float right_distance_mm;// Distancia recorrida por rueda derecha (mm)
    float left_distance_mm_acum;  // Acumulado distancia izquierda
    float right_distance_mm_acum; // Acumulado distancia derecha
} Pose;


float* promedio_adc(uint32_t adcValue[5]);
float* promedio_adc_v(uint32_t adcV[5]);
uint16_t* calculo_mm(float adc_pr[5]);
float interpolar_distancia(uint16_t adc_recibido, uint16_t *adc_tabla, float *distancias, int num_puntos);
void updatePose(Pose *pose, int32_t  pulsosL, int32_t  pulsosR, float delta_time_s);
void control_distancia(uint16_t distancia_objetivo_mm, uint16_t velocidad_objetivo_mm, uint16_t aceleracion_objetivo_mm, float distancia_actual);
float actualizar_velocidad_trapezoidal(float dist);
uint8_t destino_alcanzado(float dist);

#endif /* INC_DISTANCIA_H_ */
