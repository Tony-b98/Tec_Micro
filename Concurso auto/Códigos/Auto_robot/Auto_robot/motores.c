/*
 * motores.c - Traccion diferencial: puente H L298N + Timer0
 *
 * Timer0 en Fast PWM de 8 bits con N = 64:
 *   fPWM = 16 MHz / (64 * 256) = 976,6 Hz
 *   OCR0B -> D5 (ENA, izquierdo)    OCR0A -> D6 (ENB, derecho)
 *
 * Tabla del L298N:  IN1 IN2
 *                    1   0  avanza
 *                    0   1  retrocede
 *                    0   0  libre
 *                    1   1  freno (con EN en alto)
 */
#include <avr/io.h>
#include "motores.h"
 
#define IN1_ALTO()  (PORTD |=  (1 << PD7))
#define IN1_BAJO()  (PORTD &= ~(1 << PD7))
#define IN2_ALTO()  (PORTB |=  (1 << PB0))
#define IN2_BAJO()  (PORTB &= ~(1 << PB0))
#define IN3_ALTO()  (PORTB |=  (1 << PB4))
#define IN3_BAJO()  (PORTB &= ~(1 << PB4))
#define IN4_ALTO()  (PORTB |=  (1 << PB5))
#define IN4_BAJO()  (PORTB &= ~(1 << PB5))
 
static int16_t obj_izq = 0, obj_der = 0;    /* objetivo */
static int16_t act_izq = 0, act_der = 0;    /* aplicado (con rampa) */
static uint8_t lento = 0;
 
/* |x| de 1..255 -> pwm_min..255. Debajo de pwm_min el motor solo zumba. */
static uint8_t a_pwm(int16_t x, uint8_t pwm_min)
{
    if (x < 0) x = -x;
    if (x == 0) return 0;
    if (x > 255) x = 255;
    return (uint8_t)(pwm_min + ((uint16_t)(255 - pwm_min) * (uint16_t)x) / 255);
}
 
/* Aplica el % de compensacion de avance al PWM ya calculado */
static uint8_t ajustar(uint8_t pwm, uint8_t porcentaje)
{
    return (uint8_t)(((uint16_t)pwm * porcentaje) / 100U);
}
 
static void aplicar_izq(int16_t x)
{
    uint8_t adelante = (x > 0);
    uint8_t pwm;
    if (MOT_INVERTIR_IZQ) x = -x;
    if (x > 0)      { IN1_ALTO(); IN2_BAJO(); }
    else if (x < 0) { IN1_BAJO(); IN2_ALTO(); }
    else            { IN1_BAJO(); IN2_BAJO(); }
    pwm = a_pwm(x, MOT_PWM_MIN_IZQ);
    if (adelante) pwm = ajustar(pwm, MOT_AJUSTE_AVANCE_IZQ);
    OCR0B = pwm;
}
 
static void aplicar_der(int16_t x)
{
    uint8_t adelante = (x > 0);
    uint8_t pwm;
    if (MOT_INVERTIR_DER) x = -x;
    if (x > 0)      { IN3_ALTO(); IN4_BAJO(); }
    else if (x < 0) { IN3_BAJO(); IN4_ALTO(); }
    else            { IN3_BAJO(); IN4_BAJO(); }
    pwm = a_pwm(x, MOT_PWM_MIN_DER);
    if (adelante) pwm = ajustar(pwm, MOT_AJUSTE_AVANCE_DER);
    OCR0A = pwm;
}
 
static int16_t rampa(int16_t act, int16_t obj)
{
    if (obj > act + MOT_PASO_RAMPA) return act + MOT_PASO_RAMPA;
    if (obj < act - MOT_PASO_RAMPA) return act - MOT_PASO_RAMPA;
    return obj;
}
 
static int16_t abs16(int16_t x) { return (x < 0) ? -x : x; }
 
/* ======================================================================== */
 
void motores_init(void)
{
    DDRD |= (1 << PD5) | (1 << PD6) | (1 << PD7);
    DDRB |= (1 << PB0) | (1 << PB4) | (1 << PB5);
 
    TCCR0A = (1 << COM0A1) | (1 << COM0B1)      /* no invertido en OC0A y OC0B */
           | (1 << WGM01)  | (1 << WGM00);      /* Fast PWM 8 bits (modo 3)    */
    TCCR0B = (1 << CS01) | (1 << CS00);         /* prescaler N = 64            */
    OCR0A = 0;
    OCR0B = 0;
    aplicar_izq(0);
    aplicar_der(0);
}
 
void motores_ruedas(int16_t izq, int16_t der)
{
    /* Saturacion: si una rueda pasa de 255 se escalan las dos en la misma
       proporcion, asi la curva mantiene su forma (int32_t evita desborde). */
    int16_t m = abs16(izq);
    if (abs16(der) > m) m = abs16(der);
    if (m > 255) {
        izq = (int16_t)(((int32_t)izq * 255) / m);
        der = (int16_t)(((int32_t)der * 255) / m);
    }
    obj_izq = izq;
    obj_der = der;
}
 
void motores_mezcla(int16_t v, int16_t w)
{
    if (abs16(v) < MOT_ZONA_MUERTA) v = 0;          /* zona muerta */
    if (abs16(w) < MOT_ZONA_MUERTA) w = 0;
    if (lento) {
        v = (int16_t)(((int32_t)v * MOT_ESCALA_LENTO) / 100);
        w = (int16_t)(((int32_t)w * MOT_ESCALA_LENTO) / 100);
    }
    motores_ruedas(v + w, v - w);                   /* izq = v + w, der = v - w */
}
 
void motores_tarea(void)
{
    act_izq = rampa(act_izq, obj_izq);
    act_der = rampa(act_der, obj_der);
    aplicar_izq(act_izq);
    aplicar_der(act_der);
}
 
void motores_frenar(void)
{
    obj_izq = obj_der = act_izq = act_der = 0;
    IN1_ALTO(); IN2_ALTO(); IN3_ALTO(); IN4_ALTO();  /* freno del L298N */
    OCR0A = 255;
    OCR0B = 255;
    /* En la proxima motores_tarea() quedan libres (IN = 0 0) */
}
 
void    motores_modo_lento(uint8_t activo) { lento = activo ? 1 : 0; }
uint8_t motores_es_lento(void)             { return lento; }
int16_t motores_izq(void)                  { return act_izq; }
int16_t motores_der(void)                  { return act_der; }
 