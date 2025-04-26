/*
 * comunicacion.h
 *
 *  Created on: Apr 7, 2025
 *      Author: Andres
 */

#ifndef INC_COMUNICACION_H_
#define INC_COMUNICACION_H_

#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

typedef struct {
    uint8_t inicio;
    uint8_t tamano;
    uint8_t *datos;
    uint8_t crc;
    uint8_t fin;
} PAQUETE;

void datos_enviar(int16_t rev, int32_t contador, uint32_t tiempo, int16_t mm_s, int16_t rad_s,
                  uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4,
                  float volt1, float volt2, float volt3, float volt4,
                  uint16_t dist1, uint16_t dist2, uint16_t dist3, uint16_t dist4);
void EnviarPaquete(uint8_t *dat, uint8_t tam);
int8_t serializarPaquete(const PAQUETE* paquete, uint8_t *buffer);
uint8_t calcularCRC(uint8_t *datos, uint8_t tam);
void CDC_ReceiveCallBack(uint8_t* Buf, uint32_t Len);
uint16_t* instruction(volatile uint8_t datosRX[15]);


#endif /* INC_COMUNICACION_H_ */
