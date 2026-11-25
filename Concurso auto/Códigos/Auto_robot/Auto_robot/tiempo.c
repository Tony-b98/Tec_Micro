/*
 * tiempo.c - Base de tiempo de 1 ms con el Timer2
 *
 * Timer2 en modo CTC: 16 MHz / 64 = 250 kHz -> OCR2A = 249 -> 1 kHz.
 * El Timer0 queda para el PWM de los motores y el Timer1 para los servos.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "tiempo.h"

static volatile uint32_t ms = 0;

ISR(TIMER2_COMPA_vect)
{
    ms++;
}

void tiempo_init(void)
{
    TCCR2A = (1 << WGM21);              /* modo CTC */
    TCCR2B = (1 << CS22);               /* prescaler 64 */
    OCR2A  = 249;                       /* 250 cuentas = 1 ms */
    TIMSK2 = (1 << OCIE2A);             /* interrupcion por comparacion */
}

uint32_t millis(void)
{
    uint32_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = ms; }   /* lectura de 32 bits sin cortes */
    return t;
}
