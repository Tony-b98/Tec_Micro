#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <stdint.h>

#include "tiempo.h"
#include "motores.h"
#include "usart.h"

#define SERVO_MIN_US       1000U
#define SERVO_MAX_US       2000U
#define SERVO_INICIO_US    1500U

#define PINZA_ABIERTA_US   1400U   // apertura pinza
#define PINZA_CERRADA_US   1730U   // cierre pinza


#define PINZA_LIMITE_MIN_US 700U
#define PINZA_LIMITE_MAX_US 2300U

#define TORRE_ARRIBA_US    1100U   // altura Brazo
#define TORRE_ABAJO_US     1550U   // posicion abajo brazo

#define SERVO_PASO_US        10U
#define SERVO_PERIODO_MS     20UL

#define TRIG PD4
#define ECHO PD3

#define SONAR_ESPERA_MS    30UL
#define SONAR_ECHO_MS      40UL

#define SONAR_MIN_CM       2U
#define SONAR_MAX_CM       400U

#define FAILSAFE_MS       300UL
#define PROTO_TIMEOUT_MS   60UL
#define PROTO_DEBUG         1  // 1: confirma tramas en terminal

// ESTADOS DE LAS MAQUINAS

/* Maquina del robot (traccion) */
typedef enum
{
    EST_DETENIDO = 0,       // motores sin orden (frenan con rampa)
    EST_MOVIENDO_ASCII,     // moviendose con w,s,a,d,q,e (variable "ultimo")
    EST_MOVIENDO_BINARIO,   // moviendose con trama 0x01 (v_bin, w_bin)
    EST_FAILSAFE,           // se perdio la comunicacion: frenar y avisar
    EST_MIDIENDO            // midiendo distancia con el sonar (robot quieto)
} estado_robot_t;

/* Maquina del receptor UART */
typedef enum
{
    RX_ESPERA = 0,          // esperando inicio de trama o comando ASCII
    RX_TRAMA_BINARIA,       // juntando los 5 bytes AA CMD D1 D2 CHECK
    RX_TRAMA_HEX            // juntando 10 caracteres hex despues de ':'
} estado_rx_t;
