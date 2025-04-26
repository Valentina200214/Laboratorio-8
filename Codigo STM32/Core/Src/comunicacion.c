/*
 * comunicacion.c
 *
 *  Created on: Apr 7, 2025
 *      Author: Andres
 */

#include "comunicacion.h"
#include "motorDriver.h"
#include "velocidad.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

uint8_t buffer[63];
volatile uint8_t indexTX = 0;
volatile uint8_t datosTX[68];
PAQUETE pk1;
int8_t velocidad_motor;
int32_t valor;
float voltaje_float;
int16_t vel_mm_s;
//uint16_t objetivo_r[6] = {0};

void datos_enviar(int16_t rev, int32_t contador, uint32_t tiempo, int16_t mm_s, int16_t rad_s,
                  uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4,
                  float volt1, float volt2, float volt3, float volt4,
                  uint16_t dist1, uint16_t dist2, uint16_t dist3, uint16_t dist4) {
    int idxx = 0;

    buffer[idxx++] = 1;
    buffer[idxx++] = (rev >> 8) & 0xFF;
    buffer[idxx++] = rev & 0xFF;

    buffer[idxx++] = 2;
    buffer[idxx++] = (contador >> 24) & 0xFF;
    buffer[idxx++] = (contador >> 16) & 0xFF;
    buffer[idxx++] = (contador >> 8) & 0xFF;
    buffer[idxx++] = contador & 0xFF;


    buffer[idxx++] = 3;
    buffer[idxx++] = (tiempo >> 24) & 0xFF;
    buffer[idxx++] = (tiempo >> 16) & 0xFF;
    buffer[idxx++] = (tiempo >> 8) & 0xFF;
    buffer[idxx++] = tiempo & 0xFF;


    buffer[idxx++] = 4;
    buffer[idxx++] = (mm_s >> 8) & 0xFF;
    buffer[idxx++] = mm_s & 0xFF;


    buffer[idxx++] = 5;
    buffer[idxx++] = (rad_s >> 8) & 0xFF;
    buffer[idxx++] = rad_s & 0xFF;


    buffer[idxx++] = 6;
    buffer[idxx++] = (adc1 >> 8) & 0xFF;
    buffer[idxx++] = adc1 & 0xFF;

    buffer[idxx++] = 7;
    buffer[idxx++] = (adc2 >> 8) & 0xFF;
    buffer[idxx++] = adc2 & 0xFF;

    buffer[idxx++] = 8;
    buffer[idxx++] = (adc3 >> 8) & 0xFF;
    buffer[idxx++] = adc3 & 0xFF;

    buffer[idxx++] = 9;
    buffer[idxx++] = (adc4 >> 8) & 0xFF;
    buffer[idxx++] = adc4 & 0xFF;


    buffer[idxx++] = 10;
    uint8_t* volt1_bytes = (uint8_t*)&volt1;
    buffer[idxx++] = volt1_bytes[0];
    buffer[idxx++] = volt1_bytes[1];
    buffer[idxx++] = volt1_bytes[2];
    buffer[idxx++] = volt1_bytes[3];


    buffer[idxx++] = 11;
    uint8_t* volt2_bytes = (uint8_t*)&volt2;
    buffer[idxx++] = volt2_bytes[0];
    buffer[idxx++] = volt2_bytes[1];
    buffer[idxx++] = volt2_bytes[2];
    buffer[idxx++] = volt2_bytes[3];


    buffer[idxx++] = 12;
    uint8_t* volt3_bytes = (uint8_t*)&volt3;
    buffer[idxx++] = volt3_bytes[0];
    buffer[idxx++] = volt3_bytes[1];
    buffer[idxx++] = volt3_bytes[2];
    buffer[idxx++] = volt3_bytes[3];


    buffer[idxx++] = 13;
    uint8_t* volt4_bytes = (uint8_t*)&volt4;
    buffer[idxx++] = volt4_bytes[0];
    buffer[idxx++] = volt4_bytes[1];
    buffer[idxx++] = volt4_bytes[2];
    buffer[idxx++] = volt4_bytes[3];


    buffer[idxx++] = 14;
    buffer[idxx++] = (dist1 >> 8) & 0xFF;
    buffer[idxx++] = dist1 & 0xFF;


    buffer[idxx++] = 15;
    buffer[idxx++] = (dist2 >> 8) & 0xFF;
    buffer[idxx++] = dist2 & 0xFF;

    buffer[idxx++] = 16;
    buffer[idxx++] = (dist3 >> 8) & 0xFF;
    buffer[idxx++] = dist3 & 0xFF;

    buffer[idxx++] = 17;
    buffer[idxx++] = (dist4 >> 8) & 0xFF;
    buffer[idxx++] = dist4 & 0xFF;

    EnviarPaquete(buffer, idxx);
}

void EnviarPaquete(uint8_t *dat, uint8_t tam) {
    pk1.inicio = 0x09;
    pk1.tamano = tam + 4;
    pk1.datos = dat;
    pk1.crc = 0x00;
    pk1.fin = 0x07;

    int numDatos = serializarPaquete(&pk1, datosTX);
    CDC_Transmit_FS(datosTX, numDatos);
}

int8_t serializarPaquete(const PAQUETE* paquete, uint8_t *buffer) {
    int idx = 0;

    if (!paquete || !buffer) return -1;

    buffer[idx++] = paquete->inicio;
    buffer[idx++] = paquete->tamano;
    if (paquete->datos && paquete->tamano > 3) {
        memcpy(&buffer[idx], paquete->datos, paquete->tamano - 3);
        idx = idx + paquete->tamano - 3;
    }
    uint8_t ss = idx;
    buffer[idx++] = calcularCRC(buffer, ss);
    buffer[idx++] = paquete->fin;
    return idx;
}

uint8_t calcularCRC(uint8_t *datos, uint8_t tam) {
    uint8_t crc = 0;
    for (int i = 0; i < tam; i++) {
        crc ^= datos[i];
    }
    return crc;
}


uint16_t* instruction(volatile uint8_t datosRX[15]) {

	static uint16_t objetivo_r[6] = {0};

    	if (datosRX[0] == 0x07) {
			memcpy(&voltaje_float, &datosRX[1], sizeof(float));
			if (voltaje_float < 0) voltaje_float = 0;
			else if (voltaje_float > 7.5) voltaje_float = 7.5;
			velocidad_motor = conversor(voltaje_float);
			motores(velocidad_motor, velocidad_motor);
		} else if (datosRX[0] == 0x08) {
			vel_mm_s = (datosRX[1] << 8) | (datosRX[2]);
			velocidad_motor = conversor_mm_s(vel_mm_s);
			motores(velocidad_motor, velocidad_motor);
		} else if (datosRX[0] == 0x09) {
            valor = datosRX[1];
            velocidad_motor = valor * (-1);
            motores(velocidad_motor, velocidad_motor);
        } else if (datosRX[0] == 0x0A) {
            velocidad_motor = 100;
            motores(velocidad_motor, velocidad_motor);
        } else if (datosRX[0] == 0x0B) {
            velocidad_motor = -100;
            motores(velocidad_motor, velocidad_motor);
        } else if (datosRX[0] == 0x0C) {
            velocidad_motor = 0;
            motores(velocidad_motor, velocidad_motor);
        } else if (datosRX[0] == 0x0D) {
        	objetivo_r[0] = (datosRX[2] << 8) | datosRX[1];
            objetivo_r[1] = 1;
        } else if (datosRX[0] == 0x0E) {
        	objetivo_r[2] = (datosRX[2] << 8) | datosRX[1];
            objetivo_r[3] = 1;
        } else if (datosRX[0] == 0x0F) {
        	objetivo_r[4] = (datosRX[2] << 8) | datosRX[1];
            objetivo_r[5] = 1;
        }

    return objetivo_r;
}
