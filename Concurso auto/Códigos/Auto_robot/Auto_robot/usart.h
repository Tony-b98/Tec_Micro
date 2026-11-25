/*
 * usart.h - USART0 del ATmega328P (D0 = RX, D1 = TX)
 * Robot de transporte - Concurso Bluetooth UTEC
 *
 * En la etapa 1 se usa con el monitor serie por USB.
 * En la etapa 3 el HC-05 se conecta a estos mismos pines.
 */
#ifndef USART_H_
#define USART_H_

#include <stdint.h>
#include <avr/pgmspace.h>

void    usart_init(uint16_t baudios);         /* 9600: velocidad de datos del HC-05 */
uint8_t usart_disponible(void);               /* 1 si hay un byte recibido */
uint8_t usart_leer(void);                     /* saca un byte del buffer   */
void    usart_enviar(uint8_t dato);
void    usart_texto(const char *s);           /* texto en RAM   */
void    usart_texto_P(const char *s);         /* texto en Flash (PSTR) */
void    usart_numero(int16_t n);

#endif
