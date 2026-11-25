/*
 * usart.c - USART0 con recepcion por interrupcion y buffer circular
 *
 * 8 bits de datos, sin paridad, 1 bit de parada (8N1).
 * La ISR solo guarda el byte (minimo codigo en la ISR, clase 16).
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"

#define TAM_BUF 32                          /* potencia de 2 */
static volatile uint8_t buf[TAM_BUF];
static volatile uint8_t cabeza = 0, cola = 0;

ISR(USART_RX_vect)
{
    uint8_t d = UDR0;
    uint8_t sig = (cabeza + 1) & (TAM_BUF - 1);
    if (sig != cola) {                       /* si esta lleno se descarta */
        buf[cabeza] = d;
        cabeza = sig;
    }
}

void usart_init(uint16_t baudios)
{
    uint16_t ubrr = (uint16_t)(F_CPU / 16UL / baudios - 1);   /* 9600 -> 103 */
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    UCSR0A = 0;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);                   /* 8N1 */
}

uint8_t usart_disponible(void)
{
    return cabeza != cola;
}

uint8_t usart_leer(void)
{
    uint8_t d = 0;
    if (cabeza != cola) {
        d = buf[cola];
        cola = (cola + 1) & (TAM_BUF - 1);
    }
    return d;
}

void usart_enviar(uint8_t dato)
{
    while (!(UCSR0A & (1 << UDRE0)));        /* esperar buffer de TX libre */
    UDR0 = dato;
}

void usart_texto(const char *s)
{
    while (*s) usart_enviar((uint8_t)*s++);
}

void usart_texto_P(const char *s)
{
    char c;
    while ((c = pgm_read_byte(s++))) usart_enviar((uint8_t)c);
}

void usart_numero(int16_t n)
{
    char t[7];
    uint8_t i = 0;
    uint16_t u;
    if (n < 0) { usart_enviar('-'); u = (uint16_t)(-(int32_t)n); }
    else       { u = (uint16_t)n; }
    do { t[i++] = (char)('0' + u % 10); u /= 10; } while (u);
    while (i) usart_enviar((uint8_t)t[--i]);
}
