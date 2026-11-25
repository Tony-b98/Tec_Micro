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
static void mostrar_estado(void)
{
	usart_texto_P(PSTR(
	"\r\nESTADO DEL ROBOT\r\n"
	));

	usart_texto_P(PSTR("estado="));
	usart_texto_P(nombre_estado_P());
	usart_texto_P(PSTR(" pinza="));
	usart_texto_P(nombre_pinza_P());
	usart_texto_P(PSTR("\r\n"));

	usart_texto_P(PSTR("vel="));
	usart_numero(vel);

	usart_texto_P(PSTR(" izq="));
	usart_numero(motores_izq());

	usart_texto_P(PSTR(" der="));
	usart_numero(motores_der());

	usart_texto_P(
	motores_es_lento() ?
	PSTR(" modo=lento\r\n") :
	PSTR(" modo=rapido\r\n")
	);

	usart_texto_P(PSTR("pinza_us="));
	usart_numero(pinza_us);

	usart_texto_P(PSTR(" torre_us="));
	usart_numero(torre_us);

	usart_texto_P(PSTR("\r\n"));

	usart_texto_P(PSTR("v_bin="));
	usart_numero(v_bin);
	usart_texto_P(PSTR(" w_bin="));
	usart_numero(w_bin);
	usart_texto_P(PSTR(" ok="));
	usart_numero((int16_t)tramas_ok);
	usart_texto_P(PSTR(" errores="));
	usart_numero((int16_t)tramas_error);
	usart_texto_P(PSTR("\r\n"));

	if (sonar_valido)
	{
		usart_texto_P(PSTR("sonar="));
		usart_numero(sonar_distancia_cm);

		usart_texto_P(PSTR(" cm\r\n"));
	}
	else
	{
		usart_texto_P(PSTR(
		"sonar=sin lectura valida\r\n"
		));
	}
}

// COMANDOS ASCII (generan los eventos de las maquinas de estados)

static void procesar(char c)
{
	// Digitos 0-9: cambian la velocidad de los comandos ASCII
	if (c >= '0' && c <= '9')
	{
		vel = (int16_t)(
		((c - '0') * 255) / 9
		);

		if (estado == EST_MOVIENDO_ASCII)
		{
			aplicar_movimiento(ultimo);
		}
		mostrar_estado();
		return;
	}

	switch (c)
	{
		case 'w':
		case 's':
		case 'a':
		case 'd':
		case 'q':
		case 'e':
		// Orden de movimiento -> EST_MOVIENDO_ASCII
		v_bin = 0;
		w_bin = 0;
		ultima_orden_ms = millis();
		aplicar_movimiento(c);
		cambiar_estado(EST_MOVIENDO_ASCII);

		usart_enviar((uint8_t)c);
		usart_texto_P(PSTR("\r\n"));
		break;

		case 'x':
		// Detener con rampa -> EST_DETENIDO
		v_bin = 0;
		w_bin = 0;
		aplicar_movimiento(c);
		cambiar_estado(EST_DETENIDO);

		usart_enviar((uint8_t)c);
		usart_texto_P(PSTR("\r\n"));
		break;

		case 'f':
		// Freno inmediato -> EST_DETENIDO
		motores_frenar();
		ultimo = 'x';
		v_bin = 0;
		w_bin = 0;
		cambiar_estado(EST_DETENIDO);

		usart_texto_P(PSTR(
		"freno\r\n"
		));
		break;

		case 'l':
		case 'r':
		// Modo lento / rapido (no cambia el estado)
		motores_modo_lento(c == 'l');
		reaplicar_movimiento();
		mostrar_estado();
		break;

		case 'o':
		pinza_ordenar(0);

		usart_texto_P(PSTR(
		"ABRIENDO PINZA\r\n"
		));
		break;

		case 'c':
		pinza_ordenar(1);

		usart_texto_P(PSTR(
		"CERRANDO PINZA\r\n"
		));
		break;

		case 'u':
		torre_objetivo_us =
		limitar_pulso(TORRE_ARRIBA_US);

		usart_texto_P(PSTR(
		"SUBIENDO TORRE\r\n"
		));
		break;

		case 'b':
		torre_objetivo_us =
		limitar_pulso(TORRE_ABAJO_US);

		usart_texto_P(PSTR(
		"BAJANDO TORRE\r\n"
		));
		break;

		case 'h':
		pinza_objetivo_us = pinza_us;
		torre_objetivo_us = torre_us;
		pinza_estado = PINZA_EST_MANTENIDA;

		usart_texto_P(PSTR(
		"SERVOS: POSICION MANTENIDA\r\n"
		));
		break;

		case 't':
		// El sonar solo se usa con el robot quieto -> EST_MIDIENDO
		if (estado != EST_DETENIDO ||
		motores_izq() != 0 || motores_der() != 0)
		{
			usart_texto_P(PSTR(
			"SONAR: DETENER RUEDAS ANTES DE MEDIR\r\n"
			));
		}
		else
		{
			cambiar_estado(EST_MIDIENDO);
		}
		break;

		case '?':
		mostrar_estado();
		break;

		default:
		break;
	}
}

// PROTOCOLO BINARIO Y PRUEBAS HEXADECIMALES

static uint8_t trama[5];
static uint8_t trama_indice = 0;
static uint32_t trama_ultimo_byte_ms = 0;

static uint8_t texto_hex_nibbles = 0;
static uint8_t texto_hex_bytes[5];

static int8_t hex_valor(uint8_t c)
{
	if (c >= '0' && c <= '9') return (int8_t)(c - '0');
	if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
	if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
	return -1;
}

static void protocolo_ejecutar(uint8_t cmd, uint8_t d1, uint8_t d2)
{
	int16_t v, w;

	switch (cmd)
	{
		case 0x01: // MOVIMIENTO: v y w entre -100 y +100
		if (d1 > 200U || d2 > 200U)
		{
			tramas_error++;
			usart_texto_P(PSTR("TRAMA: MOVIMIENTO INVALIDO\r\n"));
			return;
		}

		v = ((int16_t)d1 - 100) * 255 / 100;
		w = ((int16_t)d2 - 100) * 255 / 100;

		v_bin = v;
		w_bin = w;
		ultimo = 'x';
		motores_mezcla(v, w);

		ultima_orden_ms = millis();

		// v = w = 0 es una orden de parar -> EST_DETENIDO
		cambiar_estado((v != 0 || w != 0) ?
		EST_MOVIENDO_BINARIO : EST_DETENIDO);
		break;

		case 0x02: // PINZA: d1 = 1 cerrar, d1 = 0 abrir
		if (d2 != 0U || d1 > 1U)
		{
			tramas_error++;
			usart_texto_P(PSTR("TRAMA: PINZA INVALIDA\r\n"));
			return;
		}
		pinza_ordenar(d1);
		break;

		case 0x03: // MODO: d1 = 1 lento, d1 = 0 rapido
		if (d2 != 0U || d1 > 1U)
		{
			tramas_error++;
			usart_texto_P(PSTR("TRAMA: MODO INVALIDO\r\n"));
			return;
		}
		motores_modo_lento(d1);
		reaplicar_movimiento();
		break;

		case 0x04: // TORRE (extension de nuestro robot)
		if (d2 != 0U || d1 > 1U)
		{
			tramas_error++;
			usart_texto_P(PSTR("TRAMA: TORRE INVALIDA\r\n"));
			return;
		}
		torre_objetivo_us = limitar_pulso(
		d1 ? TORRE_ABAJO_US : TORRE_ARRIBA_US);
		break;

		default:
		tramas_error++;
		usart_texto_P(PSTR("TRAMA: CMD DESCONOCIDO\r\n"));
		return;
	}

	tramas_ok++;
	#if PROTO_DEBUG
	usart_texto_P(PSTR("TRAMA OK\r\n"));
	#endif
}

static void protocolo_verificar(const uint8_t *p)
{
	if (p[0] != 0xAA)
	{
		tramas_error++;
		usart_texto_P(PSTR("TRAMA: INICIO INVALIDO\r\n"));
		return;
	}

	if ((uint8_t)(p[1] ^ p[2] ^ p[3]) != p[4])
	{
		tramas_error++;
		usart_texto_P(PSTR("TRAMA: CHECKSUM ERROR\r\n"));
		return;
	}

	protocolo_ejecutar(p[1], p[2], p[3]);
}

/* Empieza a juntar una trama binaria (ya llego el 0xAA). */
static void rx_iniciar_trama(void)
{
	trama[0] = 0xAA;
	trama_indice = 1U;
	trama_ultimo_byte_ms = millis();
	rx_estado = RX_TRAMA_BINARIA;
}

// MAQUINA DE ESTADOS DEL RECEPTOR UART

static void recibir_uart(uint8_t c)
{
	switch (rx_estado)
	{
		case RX_ESPERA:
		if (c == 0xAA)
		{
			// Inicio de trama binaria
			rx_iniciar_trama();
		}
		else if (c == ':')
		{
			// Trama de prueba escrita en hexadecimal 
			texto_hex_nibbles = 0U;
			rx_estado = RX_TRAMA_HEX;
		}
		else
		{
			// Cualquier otro caracter es un comando ASCII
			procesar((char)c);
		}
		break;

		case RX_TRAMA_BINARIA:
		// Si el byte llego tarde, la trama anterior se descarta
		if ((uint32_t)(millis() - trama_ultimo_byte_ms)
		>= PROTO_TIMEOUT_MS)
		{
			tramas_error++;
			if (c == 0xAA)
			{
				rx_iniciar_trama();
			}
			else
			{
				trama_indice = 0U;
				rx_estado = RX_ESPERA;
			}
			break;
		}

		trama[trama_indice++] = c;
		trama_ultimo_byte_ms = millis();

		if (trama_indice == 5U)
		{
			// Trama completa: volver a esperar y ejecutarla
			trama_indice = 0U;
			rx_estado = RX_ESPERA;
			protocolo_verificar(trama);
		}
		break;

		case RX_TRAMA_HEX:
		{
			// Diez caracteres hex = cinco bytes de trama
			int8_t valor = hex_valor(c);
			if (valor < 0)
			{
				texto_hex_nibbles = 0U;
				rx_estado = RX_ESPERA;
				usart_texto_P(PSTR("HEX: CARACTER INVALIDO\r\n"));
				break;
			}

			if ((texto_hex_nibbles & 1U) == 0U)
			{
				texto_hex_bytes[texto_hex_nibbles / 2U] =
				(uint8_t)((uint8_t)valor << 4);
			}
			else
			{
				texto_hex_bytes[texto_hex_nibbles / 2U] |=
				(uint8_t)valor;
			}

			texto_hex_nibbles++;
			if (texto_hex_nibbles == 10U)
			{
				texto_hex_nibbles = 0U;
				rx_estado = RX_ESPERA;
				protocolo_verificar(texto_hex_bytes);
			}
			break;
		}

		default:
		rx_estado = RX_ESPERA;
		break;
	}
}

/* Si una trama binaria quedo a medias, se descarta por tiempo. */
static void receptor_timeout(void)
{
	if (rx_estado == RX_TRAMA_BINARIA &&
	(uint32_t)(millis() - trama_ultimo_byte_ms) >= PROTO_TIMEOUT_MS)
	{
		trama_indice = 0U;
		rx_estado = RX_ESPERA;
		tramas_error++;
	}
}

// PROGRAMA PRINCIPAL

int main(void)
{
	cli();

	motores_init();

	tiempo_init();

	usart_init(9600);

	servos_init();

	sonar_init();

	sei();

	usart_texto_P(PSTR(
	"\r\n"
	"================================\r\n"
	" ROBOT UTEC - ETAPA 5\r\n"
	" TRACCION + SERVOS + ULTRASONICO\r\n"
	"================================\r\n"
	"\r\n"
	"w = Avanzar\r\n"
	"s = Retroceder\r\n"
	"a = Girar izquierda\r\n"
	"d = Girar derecha\r\n"
	"q = Curva izquierda\r\n"
	"e = Curva derecha\r\n"
	"x = Detener\r\n"
	"f = Freno\r\n"
	"0-9 = Velocidad\r\n"
	"l = Modo lento\r\n"
	"r = Modo rapido\r\n"
	"\r\n"
	"o = Abrir pinza\r\n"
	"c = Cerrar pinza\r\n"
	"u = Subir torre\r\n"
	"b = Bajar torre\r\n"
	"h = Mantener servos\r\n"
	"\r\n"
	"t = Medir distancia (detenido)\r\n"
	"? = Estado del robot\r\n"
	"FAILSAFE: 300 ms sin orden de movimiento\r\n"
	"HEX: :AA01C864AD = AVANZAR\r\n"
	"HEX: :AA01646401 = PARAR\r\n"
	"TRAMA: AA CMD D1 D2 CHECK\r\n"
	"================================\r\n"
	));

	uint32_t t_motores = millis();
	uint32_t t_servos = t_motores;

	cambiar_estado(EST_DETENIDO);

	while (1)
	{
		uint32_t ahora = millis();

		// 1) Maquina de estados del robot (incluye el failsafe)
		maquina_robot();

		// 2) Rampa de los motores cada 10 ms
		if ((uint32_t)(ahora - t_motores)
		>= 10UL)
		{
			t_motores += 10UL;
			motores_tarea();
		}

		// 3) Rampa de los servos cada 20 ms + estado de la pinza
		if ((uint32_t)(ahora - t_servos)
		>= SERVO_PERIODO_MS)
		{
			t_servos = ahora;
			servos_tarea();
			maquina_pinza();
		}

		// 4) Bytes recibidos por Bluetooth -> maquina del receptor
		uint8_t leidos = 0;
		while (usart_disponible() && leidos < 16U)
		{
			recibir_uart(usart_leer());
			leidos++;
		}

		receptor_timeout();
	}
}


