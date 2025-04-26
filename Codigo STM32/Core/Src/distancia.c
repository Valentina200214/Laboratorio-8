/*
 * distancia.c
 *
 *  Created on: Apr 7, 2025
 *      Author: Andres
 */


#include "distancia.h"
#include "velocidad.h"
#include "comunicacion.h"
#include "motorDriver.h"
#include "complemento.h"

#define ADC_BUFFER_SIZE 10
#define VELOCIDAD_INICIAL_MINIMA 100.0f
#define TOLERANCIA_MM 3


uint16_t adcMM[5] = {0};
float adcVoltaje[5] = {0};
float adc_avg[5] = {0};

uint16_t adc_buffer[5][ADC_BUFFER_SIZE] = {0};
uint8_t adc_buffer_index = 0;

uint8_t muestras = 0;
int num_puntos = 41;


/*
uint8_t perfil_iniciado = 0;

float tiempo_aceleracion = 0.0f;
float distancia_aceleracion = 0.0f;
float distancia_desaceleracion = 0.0f;
float distancia_constante = 0.0f;
float tiempo_constante = 0.0f;
float tiempo_total = 0.0f;
float distancia_objetivo_mm = 0.0f;
float velocidad_objetivo_mm = 0.0f;
float aceleracion_objetivo_mm = 0.0f;
*/
static float distancia_objetivo_mm = 0;
static float distancia_inicial_mm = 0;
static float velocidad_objetivo_mm = 0;
static float aceleracion_objetivo_mm = 0;
static float distancia_aceleracion = 0;
static float distancia_constante = 0;
static uint8_t perfil_iniciado = 0;


//uint32_t tiempo_inicio = 0;





float distancias_mm[] = {20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100,
                         105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165,
                         170, 175, 180, 185, 190, 195, 200, 205, 210, 215, 220};
uint16_t adc_sensor1[] = {2580, 2279, 1972, 1606, 1372, 1160, 998, 838, 684, 582, 506, 435,
                          373, 335, 333, 303, 277, 253, 228, 209, 193, 180, 167, 161, 155,
                          147, 140, 136, 132, 130, 128, 124, 127, 124, 122, 122, 119, 118, 118, 118, 117};
uint16_t adc_sensor2[] = {925, 849, 752, 650, 589, 517, 479, 424, 367, 333, 306, 264,
                          250, 224, 223, 205, 189, 183, 174, 161, 150, 143, 134, 130,
                          127, 122, 116, 116, 114, 113, 115, 120, 118, 121, 121, 119, 120,
                          119, 119, 119, 120};
uint16_t adc_sensor3[] = {1071, 891, 727, 557, 464, 377, 324, 274, 242, 213, 197, 173,
                          173, 166, 166, 158, 148, 144, 140, 130, 129, 124, 121, 121,
                          123, 124, 127, 124, 124, 122, 119, 118, 120, 117, 119, 122, 120,
                          119, 119, 119, 119};
uint16_t adc_sensor4[] = {3497, 3025, 2412, 1873, 1583, 1313, 1119, 969, 809, 717, 643, 575,
                          527, 492, 491, 467, 446, 427, 411, 392, 380, 374, 360, 356,
                          350, 340, 339, 334, 331, 329, 326, 324, 326, 326, 323, 321, 319,
                          319, 318, 316, 316};

//TIM_HandleTypeDef htim3;
//TIM_HandleTypeDef htim5;





float* promedio_adc(uint32_t adcValue[5]) {
    for (int i = 0; i < 5; i++) {
        adc_buffer[i][adc_buffer_index] = adcValue[i];
    }

    adc_buffer_index++;
    muestras++;

    if (muestras >= ADC_BUFFER_SIZE) {
        for (int i = 0; i < 5; i++) {
            uint32_t sum = 0;
            for (int j = 0; j < ADC_BUFFER_SIZE; j++) {
                sum += adc_buffer[i][j];
            }
            adc_avg[i] = (float)sum / ADC_BUFFER_SIZE;
        }

        adc_buffer_index = 0;
        muestras = 0;
    }

	return adc_avg;
}

float* promedio_adc_v(uint32_t adcV[5]) {

	for (int i = 0; i < 5; i++) {
		adcVoltaje[i] = (adcV[i] * 3.3) / 4095.0;
	}

	return adcVoltaje;
}

uint16_t* calculo_mm(float adc_pr[5]) {

    adcMM[0] = (uint16_t)interpolar_distancia((uint16_t)adc_pr[0], adc_sensor1, distancias_mm, num_puntos);
    adcMM[1] = (uint16_t)interpolar_distancia((uint16_t)adc_pr[1], adc_sensor2, distancias_mm, num_puntos);
    adcMM[2] = (uint16_t)interpolar_distancia((uint16_t)adc_pr[2], adc_sensor3, distancias_mm, num_puntos);
    adcMM[3] = (uint16_t)interpolar_distancia((uint16_t)adc_pr[3], adc_sensor4, distancias_mm, num_puntos);
    adcMM[4] = 0;

    return adcMM;
}

float interpolar_distancia(uint16_t adc_recibido, uint16_t *adc_tabla, float *distancias, int num_puntos) {
    if (adc_recibido >= adc_tabla[0]) {
        return distancias[0];
    }
    if (adc_recibido <= adc_tabla[num_puntos - 1]) {
        return distancias[num_puntos - 1];
    }

    for (int i = 0; i < num_puntos - 1; i++) {
        if (adc_recibido <= adc_tabla[i] && adc_recibido >= adc_tabla[i + 1]) {
            float x0 = adc_tabla[i];
            float x1 = adc_tabla[i + 1];
            float y0 = distancias[i];
            float y1 = distancias[i + 1];

            float m = (y1 - y0) / (x1 - x0);
            float b = y0 - m * x0;
            return m * adc_recibido + b;
        }
    }

    return distancias[num_puntos - 1];
}



void updatePose(Pose *pose, int32_t  pulsosL, int32_t  pulsosR, float delta_time_s){

    pose->left_distance_mm = (2.0 * PI * RADIO_RUEDA_MM * (float)pulsosL) / PULSOS_POR_REVOLUCION;
    pose->right_distance_mm = (2.0 * PI * RADIO_RUEDA_MM * (float)pulsosR) / PULSOS_POR_REVOLUCION;
    pose->left_distance_mm_acum += pose->left_distance_mm;
    pose->right_distance_mm_acum += pose->right_distance_mm;

    // Distancia promedio recorrida
    float delta_lineal = (pose->left_distance_mm + pose->right_distance_mm) / 2.0;
    pose->avanceLineal += delta_lineal;
    pose->velLineal = delta_lineal / delta_time_s;  // Velocidad en mm/s
    pose->velMotorL = pose->left_distance_mm / delta_time_s;
    pose->velMotorR = pose->right_distance_mm / delta_time_s;

    float delta_angular = (pose->right_distance_mm - pose->left_distance_mm) / DISTANCIA_ENTRE_RUEDAS_MM;
    pose->velAngular = delta_angular / delta_time_s;  // Velocidad angular en rad/s

    pose->theta += delta_angular;
    if(pose->theta > M_PI) pose->theta -= 2.0 * M_PI;
    else if(pose->theta < -M_PI) pose->theta += 2.0 * M_PI;
    //pose.theta = fmod(pose.theta, 2.0 * M_PI);  // Normalizar ángulo

    //pose.x += delta_lineal * cos(pose.theta);
    //pose.y += delta_lineal * sin(pose.theta);

}


/*
void control_distancia(uint16_t distancia_objetivo_mm, uint16_t velocidad_objetivo_mm, uint16_t aceleracion_objetivo_mm) {

	distancia_objetivo_mm = (float)distancia_objetivo_mm_;
	velocidad_objetivo_mm = (float)velocidad_objetivo_mm_;
	aceleracion_objetivo_mm = (float)aceleracion_objetivo_mm_;

    tiempo_aceleracion = (float)velocidad_objetivo_mm / aceleracion_objetivo_mm;
    distancia_aceleracion = 0.5f * aceleracion_objetivo_mm * tiempo_aceleracion * tiempo_aceleracion;
    distancia_desaceleracion = distancia_aceleracion;
    distancia_constante = distancia_objetivo_mm - 2 * distancia_aceleracion;
    tiempo_constante = (distancia_constante > 0) ? distancia_constante / velocidad_objetivo_mm : 0;
    tiempo_total = 2 * tiempo_aceleracion + tiempo_constante;

    if (distancia_constante <= 0) {
		float velocidad_max_ajustada = sqrtf(2.0f * aceleracion_objetivo_mm * (distancia_objetivo_mm / 2.0f));
		tiempo_aceleracion = velocidad_max_ajustada / aceleracion_objetivo_mm;
		distancia_aceleracion = distancia_objetivo_mm / 2.0f;
		distancia_desaceleracion = distancia_aceleracion;
		tiempo_constante = 0;
		tiempo_total = 2 * tiempo_aceleracion;
		velocidad_objetivo_mm = (uint16_t)velocidad_max_ajustada;
	}

}
*/


/*
void control_distancia(uint16_t distancia_objetivo_mm_, uint16_t velocidad_objetivo_mm_, uint16_t aceleracion_objetivo_mm_, float distancia_actual) {
    distancia_objetivo_mm = (float)distancia_objetivo_mm_;
    velocidad_objetivo_mm = (float)velocidad_objetivo_mm_;
    aceleracion_objetivo_mm = (float)aceleracion_objetivo_mm_;

    float tiempo_aceleracion = velocidad_objetivo_mm / aceleracion_objetivo_mm;
    distancia_aceleracion = 0.5f * aceleracion_objetivo_mm * tiempo_aceleracion * tiempo_aceleracion;
    distancia_constante = distancia_objetivo_mm - 2 * distancia_aceleracion;

    if (distancia_constante <= 0) {
        float velocidad_max_ajustada = sqrtf(2.0f * aceleracion_objetivo_mm * (distancia_objetivo_mm / 2.0f));
        tiempo_aceleracion = velocidad_max_ajustada / aceleracion_objetivo_mm;
        distancia_aceleracion = distancia_objetivo_mm / 2.0f;
        distancia_constante = 0;
        velocidad_objetivo_mm = velocidad_max_ajustada;
    }

    perfil_iniciado = 0;
}
*/


void control_distancia(uint16_t distancia_objetivo_mm_, uint16_t velocidad_objetivo_mm_, uint16_t aceleracion_objetivo_mm_, float distancia_actual) {
    distancia_inicial_mm = distancia_actual;
    distancia_objetivo_mm = distancia_actual + (float)distancia_objetivo_mm_;
    velocidad_objetivo_mm = (float)velocidad_objetivo_mm_;
    aceleracion_objetivo_mm = (float)aceleracion_objetivo_mm_;

    float tiempo_aceleracion = velocidad_objetivo_mm / aceleracion_objetivo_mm;
    distancia_aceleracion = 0.5f * aceleracion_objetivo_mm * tiempo_aceleracion * tiempo_aceleracion;
    distancia_constante = (distancia_objetivo_mm - distancia_inicial_mm) - 2 * distancia_aceleracion;

    if (distancia_constante <= 0) {
        float velocidad_max_ajustada = sqrtf(2.0f * aceleracion_objetivo_mm * ((distancia_objetivo_mm - distancia_inicial_mm) / 2.0f));
        tiempo_aceleracion = velocidad_max_ajustada / aceleracion_objetivo_mm;
        distancia_aceleracion = (distancia_objetivo_mm - distancia_inicial_mm) / 2.0f;
        distancia_constante = 0;
        velocidad_objetivo_mm = velocidad_max_ajustada;
    }

    perfil_iniciado = 0;
}
/*
float actualizar_velocidad_trapezoidal(float dist) {
    if (!perfil_iniciado) {
        perfil_iniciado = 1;
    }

    float velocidad_deseada = 0.0f;


    // Fase de aceleración
	if (dist < distancia_aceleracion) {
		if (dist < 1.0f) {
			velocidad_deseada = VELOCIDAD_INICIAL_MINIMA;
		} else {
			velocidad_deseada = sqrtf(2.0f * aceleracion_objetivo_mm * dist);
		}
	}

    // Fase de velocidad constante
    else if (dist < (distancia_aceleracion + distancia_constante)) {
        velocidad_deseada = velocidad_objetivo_mm;
    }

    // Fase de desaceleración
    else if (dist < distancia_objetivo_mm) {
        float distancia_restante = distancia_objetivo_mm - dist;
        velocidad_deseada = sqrtf(2.0f * aceleracion_objetivo_mm * distancia_restante);
    }

    else {
        velocidad_deseada = 0.0f;
    }

    if (velocidad_deseada < 0) {
        velocidad_deseada = 0.0f;
    }

    return velocidad_deseada;
}
*/

float actualizar_velocidad_trapezoidal(float dist) {
    // Iniciar el perfil si no ha comenzado
    if (!perfil_iniciado) {
        perfil_iniciado = 1;
    }

    // Calcular la distancia recorrida relativa desde el inicio del movimiento
    float dist_relativa = dist - distancia_inicial_mm;
    float velocidad_deseada = 0.0f;

    // Fase de aceleración
    if (dist_relativa < distancia_aceleracion) {
        // Si dist_relativa es muy pequeño, usar velocidad inicial mínima para arrancar
        if (dist_relativa < 1.0f) { // Umbral de 1 mm para evitar dist = 0
            velocidad_deseada = VELOCIDAD_INICIAL_MINIMA;
        } else {
            velocidad_deseada = sqrtf(2.0f * aceleracion_objetivo_mm * dist_relativa);
        }
    }
    // Fase de velocidad constante
    else if (dist_relativa < (distancia_aceleracion + distancia_constante)) {
        velocidad_deseada = velocidad_objetivo_mm;
    }
    // Fase de desaceleración
    else if (dist_relativa < (distancia_objetivo_mm - distancia_inicial_mm)) {
        float distancia_restante = (distancia_objetivo_mm - distancia_inicial_mm) - dist_relativa;
        velocidad_deseada = sqrtf(2.0f * aceleracion_objetivo_mm * distancia_restante);
    }
    // Después de alcanzar la distancia objetivo
    else {
        velocidad_deseada = 0.0f;
    }

    // Asegurar que la velocidad no sea negativa
    if (velocidad_deseada < 0) {
        velocidad_deseada = 0.0f;
    }

    return velocidad_deseada;
}


uint8_t destino_alcanzado(float dist) {

	if (fabs(dist - distancia_objetivo_mm) <= TOLERANCIA_MM) {

		perfil_iniciado = 0;
        return 1;
    }
    return 0;
}









/*
void control_distancia(uint16_t distancia_objetivo_mm, uint16_t velocidad_objetivo_mm, uint16_t aceleracion_objetivo_mm) {

    const float perimetro_rueda_mm = PI * DIAMETRO_RUEDA_MM;

    float revoluciones = (float)distancia_objetivo_mm / perimetro_rueda_mm;
    int32_t pulsos_objetivo = (int32_t)(revoluciones * PULSOS_POR_REVOLUCION); // 1431 pulsos por revolución

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    pulsos_izq_anterior = 0;
    pulsos_der_anterior = 0;

    conversor_mm_s(velocidad_objetivo_mm);
    int8_t pwm = pwm_mm;

    motores(pwm, pwm);

    int32_t pulsos_izq = 0;
    int32_t pulsos_der = 0;
    while ((pulsos_izq < pulsos_objetivo) || (pulsos_der < pulsos_objetivo)) {

    	pulsos_izq = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        pulsos_der = (int32_t)__HAL_TIM_GET_COUNTER(&htim3);
    }

    motores(0, 0);
}

*/

/*
void control_distancia(uint16_t distancia_objetivo_mm, uint16_t velocidad_objetivo_mm, uint16_t aceleracion_objetivo_mm) {

    distancia_objetivo_mm = distancia_objetivo_mm;
    velocidad_objetivo_mm = velocidad_objetivo_mm;
    aceleracion_objetivo_mm = aceleracion_objetivo_mm;


    t_actual = 0.0f;
    v_actual = 0.0f;
    pwm_actual = 0;
    pulsos_totales = 0;
    fase = 1;


    pulsos_objetivo = (int32_t)(distancia_objetivo_mm * pulsos_por_mm);

    tim_encoder_1_Global->Instance->CNT = 0;
    tim_encoder_2_Global->Instance->CNT = 0;


    float v_max = (float)velocidad_objetivo_mm;
    float a = (float)aceleracion_objetivo_mm;
    t_acc = v_max / a; // Tiempo para alcanzar v_max
    float d_acc = 0.5f * a * t_acc * t_acc;
    t_dec = t_acc; // Tiempo de desaceleración (simétrico)
    float d_dec = d_acc; // Distancia durante desaceleración
    float d_total = (float)distancia_objetivo_mm;

    t_const = 0.0f;
    if (d_acc + d_dec <= d_total) {
        // Perfil trapezoidal
        float d_const = d_total - (d_acc + d_dec);
        t_const = d_const / v_max;
    } else {
        // Perfil triangular: ajustar v_max
        v_max = sqrtf(a * d_total);
        t_acc = v_max / a;
        t_dec = t_acc;
        t_const = 0.0f;
    }

    t_total = t_acc + t_const + t_dec;
}
 */





