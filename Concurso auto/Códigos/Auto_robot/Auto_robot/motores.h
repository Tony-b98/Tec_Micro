/*
 * motores.h - Traccion diferencial: puente H L298N + Timer0
 * Robot de transporte - Concurso Bluetooth UTEC
 *
 * Conexiones (Arduino Uno -> L298N):
 *   D5  (PD5, OC0B) -> ENA  motor izquierdo (PWM)   * sacar el jumper de ENA
 *   D7  (PD7)       -> IN1
 *   D8  (PB0)       -> IN2
 *   D6  (PD6, OC0A) -> ENB  motor derecho (PWM)     * sacar el jumper de ENB
 *   D12 (PB4)       -> IN3
 *   D13 (PB5)       -> IN4
 *   GND             -> GND del L298N y de la bateria (tierra comun)
 *
 * Velocidades en el rango -255..255 (el signo es el sentido).
 * Mezcla diferencial (clase 19):  izq = v + w   der = v - w
 */
#ifndef MOTORES_H_
#define MOTORES_H_
 
#include <stdint.h>
 
/* ---- Calibracion: MEDIR y anotar en el README ---- */
#define MOT_PWM_MIN_IZQ   90    /* PWM con el que arranca el motor izquierdo */
#define MOT_PWM_MIN_DER   90    /* PWM con el que arranca el motor derecho   */
#define MOT_INVERTIR_IZQ  0     /* 1 si el motor izquierdo gira al reves     */
#define MOT_INVERTIR_DER  0     /* 1 si el motor derecho gira al reves       */
#define MOT_ZONA_MUERTA   15    /* |v| o |w| menor que esto se toma como 0   */
#define MOT_PASO_RAMPA    10    /* cambio maximo por cada 10 ms (0 a 255 en 0,26 s) */
#define MOT_ESCALA_LENTO  50    /* modo lento: v y w al 50 %                 */
 
/* Compensacion SOLO al avanzar (en %, 100 = sin cambio).
 * El motor derecho (OUT3/OUT4, ENB) gira mas rapido hacia adelante y el
 * auto se desvia. Se le baja el PWM solo cuando esa rueda va hacia adelante.
 * Si el auto sigue doblando hacia el lado del motor izquierdo -> bajar
 * MOT_AJUSTE_AVANCE_DER (ej. 88). Si ahora dobla al reves -> subirlo (ej. 95).
 * Cambiar de a 2-3 puntos. Marcha atras no se toca. */
#define MOT_AJUSTE_AVANCE_IZQ  100
#define MOT_AJUSTE_AVANCE_DER   91
 
void    motores_init(void);
void    motores_mezcla(int16_t v, int16_t w);   /* v: avanzar, w: girar (-255..255) */
void    motores_ruedas(int16_t izq, int16_t der);
void    motores_tarea(void);                    /* llamar cada 10 ms (rampa)  */
void    motores_frenar(void);                   /* freno activo inmediato     */
void    motores_modo_lento(uint8_t activo);     /* 1 = lento (pinza), 0 = rapido */
uint8_t motores_es_lento(void);
int16_t motores_izq(void);                      /* valor aplicado, para telemetria */
int16_t motores_der(void);
 
#endif
 