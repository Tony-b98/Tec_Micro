/*
 * tiempo.h - Base de tiempo de 1 ms con el Timer2
 * Robot de transporte - Concurso Bluetooth UTEC
 */
#ifndef TIEMPO_H_
#define TIEMPO_H_

#include <stdint.h>

void     tiempo_init(void);   /* configura el Timer2 y arranca la cuenta */
uint32_t millis(void);        /* milisegundos desde el arranque */

#endif
