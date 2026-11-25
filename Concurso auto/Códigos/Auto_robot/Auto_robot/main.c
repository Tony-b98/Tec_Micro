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

/* Maquina de la pinza */
typedef enum
{
	PINZA_EST_INICIO = 0,   // posicion central al encender
	PINZA_EST_ABRIENDO,     // servo en movimiento hacia abierta
	PINZA_EST_ABIERTA,      // llego a la posicion abierta
	PINZA_EST_CERRANDO,     // servo en movimiento hacia cerrada
	PINZA_EST_CERRADA,      // llego a la posicion cerrada (agarre)
	PINZA_EST_MANTENIDA     // comando 'h': quieta donde estaba
} estado_pinza_t;

static estado_robot_t estado = EST_DETENIDO;
static estado_rx_t    rx_estado = RX_ESPERA;
static estado_pinza_t pinza_estado = PINZA_EST_INICIO;

// VARIABLES DE TRACCION

static int16_t vel = 128;       // velocidad para comandos ASCII (0-255)
static char ultimo = 'x';       // ultimo comando ASCII de movimiento

static uint32_t ultima_orden_ms = 0;   // para el failsafe

static int16_t v_bin = 0;       // avance pedido por trama binaria
static int16_t w_bin = 0;       // giro pedido por trama binaria

static uint16_t tramas_ok = 0;
static uint16_t tramas_error = 0;

// VARIABLES DE SERVOS Y SONAR

static uint16_t pinza_us = SERVO_INICIO_US;
static uint16_t pinza_objetivo_us = SERVO_INICIO_US;

static uint16_t torre_us = TORRE_ARRIBA_US;          // al encender, brazo arriba
static uint16_t torre_objetivo_us = TORRE_ARRIBA_US;

static uint16_t sonar_distancia_cm = 0;
static uint8_t sonar_valido = 0;

static volatile uint32_t timer1_base = 0;

ISR(TIMER1_OVF_vect)
{
	timer1_base += 40000UL;
}

static uint32_t timer1_ticks(void)
{
	uint32_t tiempo;
	uint32_t base;
	uint16_t cuenta;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		cuenta = TCNT1;
		base = timer1_base;

		if ((TIFR1 & (1 << TOV1)) &&
		cuenta < 20000U)
		{
			base += 40000UL;
		}

		tiempo = base + (uint32_t)cuenta;
	}

	return tiempo;
}

static void servos_init(void)
{
	PRR &= ~(1 << PRTIM1);

	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;

	TIMSK1 = 0;

	DDRB |= (1 << PB1) |
	(1 << PB2);

	TCNT1 = 0;

	timer1_base = 0;

	ICR1 = 39999U;

	OCR1A = (pinza_us * 2U) - 1U;
	OCR1B = (torre_us * 2U) - 1U;

	TIFR1 = (1 << ICF1) |
	(1 << OCF1A) |
	(1 << OCF1B) |
	(1 << TOV1);

	TIMSK1 = (1 << TOIE1);

	TCCR1A = (1 << COM1A1) |
	(1 << COM1B1) |
	(1 << WGM11);

	TCCR1B = (1 << WGM13) |
	(1 << WGM12) |
	(1 << CS11);
}

static uint16_t limitar_pulso(uint16_t pulso)
{
	if (pulso < SERVO_MIN_US)
	{
		return SERVO_MIN_US;
	}

	if (pulso > SERVO_MAX_US)
	{
		return SERVO_MAX_US;
	}

	return pulso;
}

static uint16_t limitar_pinza(uint16_t pulso)
{
	if (pulso < PINZA_LIMITE_MIN_US)
	{
		return PINZA_LIMITE_MIN_US;
	}

	if (pulso > PINZA_LIMITE_MAX_US)
	{
		return PINZA_LIMITE_MAX_US;
	}

	return pulso;
}

static uint16_t acercar_servo(
uint16_t actual,
uint16_t objetivo)
{
	if (actual < objetivo)
	{
		uint16_t diferencia = objetivo - actual;

		if (diferencia <= SERVO_PASO_US)
		{
			return objetivo;
		}

		return actual + SERVO_PASO_US;
	}

	if (actual > objetivo)
	{
		uint16_t diferencia = actual - objetivo;

		if (diferencia <= SERVO_PASO_US)
		{
			return objetivo;
		}

		return actual - SERVO_PASO_US;
	}

	return actual;
}

static void servos_tarea(void)
{
	pinza_us = acercar_servo(
	pinza_us,
	pinza_objetivo_us
	);

	torre_us = acercar_servo(
	torre_us,
	torre_objetivo_us
	);

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		OCR1A = (pinza_us * 2U) - 1U;
		OCR1B = (torre_us * 2U) - 1U;
	}
}

static void sonar_init(void)
{
	DDRD |= (1 << TRIG);

	DDRD &= ~(1 << ECHO);

	PORTD &= ~(1 << ECHO);

	PORTD &= ~(1 << TRIG);
}

static void sonar_enviar_u32(uint32_t numero)
{
	char buffer[11];
	uint8_t i = 0;

	if (numero == 0)
	{
		usart_enviar('0');
		return;
	}

	while (numero > 0)
	{
		buffer[i++] =
		(char)('0' + numero % 10UL);

		numero /= 10UL;
	}

	while (i > 0)
	{
		usart_enviar(
		(uint8_t)buffer[--i]
		);
	}
}

static void sonar_medir(void)
{
	uint32_t inicio_espera;
	uint32_t inicio_alto;

	uint32_t ticks_inicio;
	uint32_t ticks_fin;
	uint32_t ticks;
	uint32_t distancia;

	sonar_valido = 0;

	if (PIND & (1 << ECHO))
	{
		usart_texto_P(PSTR(
		"SONAR: ECHO YA ESTA ALTO\r\n"
		));
		return;
	}

	PORTD &= ~(1 << TRIG);
	_delay_us(3);

	PORTD |= (1 << TRIG);
	_delay_us(10);

	PORTD &= ~(1 << TRIG);

	inicio_espera = millis();

	while (!(PIND & (1 << ECHO)))
	{
		if ((uint32_t)(
		millis() - inicio_espera
		) >= SONAR_ESPERA_MS)
		{
			usart_texto_P(PSTR(
			"SONAR: NO LLEGA ECHO\r\n"
			));
			return;
		}
	}

	ticks_inicio = timer1_ticks();

	inicio_alto = millis();

	while (PIND & (1 << ECHO))
	{
		if ((uint32_t)(
		millis() - inicio_alto
		) >= SONAR_ECHO_MS)
		{
			usart_texto_P(PSTR(
			"SONAR: ECHO DEMASIADO LARGO\r\n"
			));
			return;
		}
	}

	ticks_fin = timer1_ticks();

	ticks = ticks_fin - ticks_inicio;

	distancia = (ticks + 58UL) / 116UL;

	usart_texto_P(PSTR(
	"\r\nSONAR TICKS: "
	));

	sonar_enviar_u32(ticks);

	usart_texto_P(PSTR(
	"\r\nDISTANCIA: "
	));

	sonar_enviar_u32(distancia);

	usart_texto_P(PSTR(
	" cm\r\n"
	));

	if (distancia >= SONAR_MIN_CM &&
	distancia <= SONAR_MAX_CM)
	{
		sonar_distancia_cm =
		(uint16_t)distancia;

		sonar_valido = 1;

		usart_texto_P(PSTR(
		"SONAR: LECTURA VALIDA\r\n"
		));
	}
	else
	{
		usart_texto_P(PSTR(
		"SONAR: FUERA DE RANGO\r\n"
		));
	}
}

// CONTROL DE TRACCION

/* Traduce un comando ASCII de movimiento a (v, w) para los motores. */
static void aplicar_movimiento(char c)
{
	int16_t giro = (vel * 3) / 4;

	switch (c)
	{
		case 'w':                           // avanzar
		motores_mezcla(vel, 0);
		break;

		case 's':                           // retroceder
		motores_mezcla(-vel, 0);
		break;

		case 'a':                           // girar izquierda
		motores_mezcla(0, -giro);
		break;

		case 'd':                           // girar derecha
		motores_mezcla(0, giro);
		break;

		case 'q':                           // curva izquierda
		motores_mezcla(vel, -vel / 2);
		break;

		case 'e':                           // curva derecha
		motores_mezcla(vel, vel / 2);
		break;

		case 'x':                           // detener (con rampa)
		motores_mezcla(0, 0);
		break;

		default:
		return;
	}

	ultimo = c;
}

/* Vuelve a aplicar la orden actual (al cambiar velocidad o modo lento). */
static void reaplicar_movimiento(void)
{
	switch (estado)
	{
		case EST_MOVIENDO_ASCII:
		aplicar_movimiento(ultimo);
		break;

		case EST_MOVIENDO_BINARIO:
		motores_mezcla(v_bin, w_bin);
		break;

		default:                            // detenido: no hay nada que reaplicar
		break;
	}
}

// MAQUINA DE ESTADOS DEL ROBOT

/* Unico punto donde se cambia el estado del robot. */
static void cambiar_estado(estado_robot_t nuevo)
{
    estado = nuevo;
}

/*
 * Se llama en cada vuelta del lazo principal.
 * Cada case describe lo que el robot hace en ese estado y
 * cuando pasa al siguiente.
 */
static void maquina_robot(void)
{
    switch (estado)
    {
        case EST_DETENIDO:
            // Nada que hacer: motores_tarea() lleva las ruedas a 0
            // con rampa. Se sale de aqui cuando llega un comando.
            break;

        case EST_MOVIENDO_ASCII:
        case EST_MOVIENDO_BINARIO:
            // FAILSAFE: la app manda ordenes seguido; si pasan
            // FAILSAFE_MS sin ninguna, se perdio el Bluetooth.
            if ((uint32_t)(millis() - ultima_orden_ms) >= FAILSAFE_MS)
            {
                cambiar_estado(EST_FAILSAFE);
            }
            break;

        case EST_FAILSAFE:
            // Freno inmediato y se olvida la ultima orden.
            motores_frenar();
            ultimo = 'x';
            v_bin = 0;
            w_bin = 0;

            usart_texto_P(PSTR("FAILSAFE: MOTORES DETENIDOS\r\n"));

            cambiar_estado(EST_DETENIDO);
            break;

        case EST_MIDIENDO:
            // La medicion es corta (< 70 ms) y se hace de una vez.
            sonar_medir();
            cambiar_estado(EST_DETENIDO);
            break;

        default:
            // Estado desconocido (no deberia pasar): parar por seguridad.
            cambiar_estado(EST_FAILSAFE);
            break;
    }
}

// MAQUINA DE ESTADOS DE LA PINZA

/* Ordena abrir o cerrar y pasa al estado "en movimiento". */
static void pinza_ordenar(uint8_t cerrar)
{
    if (cerrar)
    {
        pinza_objetivo_us = limitar_pinza(PINZA_CERRADA_US);
        pinza_estado = PINZA_EST_CERRANDO;
    }
    else
    {
        pinza_objetivo_us = limitar_pinza(PINZA_ABIERTA_US);
        pinza_estado = PINZA_EST_ABRIENDO;
    }
}

/* Se llama despues de mover los servos: detecta cuando la pinza llego. */
static void maquina_pinza(void)
{
    switch (pinza_estado)
    {
        case PINZA_EST_ABRIENDO:
            if (pinza_us == pinza_objetivo_us)
            {
                pinza_estado = PINZA_EST_ABIERTA;
                usart_texto_P(PSTR("PINZA ABIERTA\r\n"));
            }
            break;

        case PINZA_EST_CERRANDO:
            if (pinza_us == pinza_objetivo_us)
            {
                pinza_estado = PINZA_EST_CERRADA;
                usart_texto_P(PSTR("PINZA CERRADA\r\n"));
            }
            break;

        case PINZA_EST_INICIO:
        case PINZA_EST_ABIERTA:
        case PINZA_EST_CERRADA:
        case PINZA_EST_MANTENIDA:
        default:
            // Quieta: espera un comando nuevo.
            break;
    }
}

static const char *nombre_estado_P(void)
{
    switch (estado)
    {
        case EST_DETENIDO:          return PSTR("DETENIDO");
        case EST_MOVIENDO_ASCII:    return PSTR("MOVIENDO_ASCII");
        case EST_MOVIENDO_BINARIO:  return PSTR("MOVIENDO_BINARIO");
        case EST_FAILSAFE:          return PSTR("FAILSAFE");
        case EST_MIDIENDO:          return PSTR("MIDIENDO");
        default:                    return PSTR("?");
    }
}

static const char *nombre_pinza_P(void)
{
    switch (pinza_estado)
    {
        case PINZA_EST_INICIO:      return PSTR("INICIO");
        case PINZA_EST_ABRIENDO:    return PSTR("ABRIENDO");
        case PINZA_EST_ABIERTA:     return PSTR("ABIERTA");
        case PINZA_EST_CERRANDO:    return PSTR("CERRANDO");
        case PINZA_EST_CERRADA:     return PSTR("CERRADA");
        case PINZA_EST_MANTENIDA:   return PSTR("MANTENIDA");
        default:                    return PSTR("?");
    }
}
